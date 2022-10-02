# sys
import os
# inference
import torch
import numpy as np
import cv2
from fairseq import utils, tasks
from fairseq import checkpoint_utils
from fairseq import options
from fairseq.dataclass.utils import convert_namespace_to_omegaconf
from utils.eval_utils import eval_step
from utils.zero_shot_utils import zero_shot_step
from tasks.mm_tasks.caption import CaptionTask
from tasks.mm_tasks.refcoco import RefcocoTask
from tasks.mm_tasks.vqa_gen import VqaGenTask
from models.ofa import OFAModel
from torchvision import transforms
from PIL import Image

# web server
from flask import Flask, request, Response
import json
import base64

PARAM_DIR = "/var/mount/params"
# OFA_PARAM[TASK][SCALE]
OFA_PARAM = {
    "caption":{
        "large":"caption_large_best_clean.pt",
        "huge":"caption_huge_best.pt"
    },
    "refcoco":{
        "large":"refcocog_large_best.pt",
        "huge":"refcocog_large_best.pt"
    },
    "vqa_gen":{
        "large":"vqa_large_best.pt",
        "huge":"vqa_large_best.pt"
    }
}

def apply_half(t):
    if t.dtype is torch.float32:
        return t.to(dtype=torch.half)
    return t

class Inference:
    def __init__(self, task, model_scale):
        self.use_cuda = torch.cuda.is_available()
        self.use_fp16 = False

        # set params
        param = OFA_PARAM[task][model_scale]
        param_path = os.path.join(PARAM_DIR, param)
        overrides={"bpe_dir":"utils/BPE", "eval_cider":False, "beam":5,
                   "max_len_b":16, "no_repeat_ngram_size":3, "seed":7}

        self.task_name = task
        if task == "caption":
            tasks.register_task(task, CaptionTask)
            self.models, self.cfg, self.task = checkpoint_utils.load_model_ensemble_and_task(
                utils.split_paths(param_path),
                arg_overrides=overrides)
        elif task == "refcoco":
            tasks.register_task(self.task, RefcocoTask)
            self.models, self.cfg, self.task = checkpoint_utils.load_model_ensemble_and_task(
                utils.split_paths(param_path),
                arg_overrides=overrides)
            self.cfg.common.seed = 7
            self.cfg.generation.beam = 5
            self.cfg.generation.min_len = 4
            self.cfg.generation.max_len_a = 0
            self.cfg.generation.max_len_b = 4
            self.cfg.generation.no_repeat_ngram_size = 3
            if self.cfg.common.seed is not None and not self.cfg.generation.no_seed_provided:
                np.random.seed(self.cfg.common.seed)
                utils.set_torch_seed(self.cfg.common.seed)
        elif task == "vqa_gen":
            tasks.register_task('vqa_gen', VqaGenTask)
            parser = options.get_generation_parser()
            input_args = ["", "--task=vqa_gen", "--beam=100", "--unnormalized", "--path={}".format(param_path), "--bpe-dir=utils/BPE"]
            args = options.parse_args_and_arch(parser, input_args)
            cfg = convert_namespace_to_omegaconf(args)
            self.task = tasks.setup_task(cfg.task)
            self.models, self.cfg = checkpoint_utils.load_model_ensemble(
                utils.split_paths(cfg.common_eval.path),
                task=self.task)
        else:
            raise RuntimeError("Please select models from caption, refcoco, vqa_gen")
            return

        # Move models to GPU
        for model in self.models:
            model.eval()
            if self.use_fp16:
                model.half()
            if self.use_cuda and not self.cfg.distributed_training.pipeline_model_parallel:
                model.cuda()
            model.prepare_for_inference_(self.cfg)

        # Image transform
        self.generator = self.task.build_generator(self.models, self.cfg.generation)
        mean = [0.5, 0.5, 0.5]
        std = [0.5, 0.5, 0.5]
        self.patch_resize_transform = transforms.Compose([
            lambda image: image.convert("RGB"),
            transforms.Resize((self.cfg.task.patch_image_size, self.cfg.task.patch_image_size), interpolation=Image.BICUBIC),
            transforms.ToTensor(),
            transforms.Normalize(mean=mean, std=std),
        ])

        self.pad_idx = self.task.src_dict.pad()

    def visual_grounding(self, Image, Text):
        sample = self.construct_sample(Image, Text.lower())
        sample = utils.move_to_cuda(sample) if self.use_cuda else sample
        sample = utils.apply_to_sample(apply_half, sample) if self.use_fp16 else sample
        with torch.no_grad():
            result, scores = eval_step(self.task, self.generator, self.models, sample)
        img = np.asarray(Image)
        cv2.rectangle(
            img,
            (int(result[0]["box"][0]), int(result[0]["box"][1])),
            (int(result[0]["box"][2]), int(result[0]["box"][3])),
            (0, 255, 0), 3)
        return img

    def encode_text(self, text, length=None, append_bos=False, append_eos=False):
        bos_item = torch.LongTensor([self.task.src_dict.bos()])
        eos_item = torch.LongTensor([self.task.src_dict.eos()])
        # pad_idx = self.task.src_dict.pad()
        s = self.task.tgt_dict.encode_line(
            line=self.task.bpe.encode(text),
            add_if_not_exist=False,
            append_eos=False).long()
        if length is not None:
            s = s[:length]
        if append_bos:
            s = torch.cat([bos_item, s])
        if append_eos:
            s = torch.cat([s, eos_item])
        return s

    def construct_sample(self, image, text):
        if self.task_name == "caption" or self.task_name == "vqa_gen":
            patch_image = self.patch_resize_transform(image).unsqueeze(0)
            patch_mask = torch.tensor([True])
            src_text = self.encode_text(" " + text, append_bos=True, append_eos=True).unsqueeze(0)
            src_length = torch.LongTensor([s.ne(self.pad_idx).long().sum() for s in src_text])
            if self.task_name == "caption":
                sample = {
                    "id":np.array(['42']),
                    "net_input": {
                        "src_tokens": src_text,
                        "src_lengths": src_length,
                        "patch_images": patch_image,
                        "patch_masks": patch_mask
                    }
                }
            elif self.task_name == "vqa_gen":
                ref_dict = np.array([{'yes': 1.0}]) # just placeholder
                sample = {
                    "id":np.array(['42']),
                    "net_input": {
                        "src_tokens": src_text,
                        "src_lengths": src_length,
                        "patch_images": patch_image,
                        "patch_masks": patch_mask
                    },
                    "ref_dict": ref_dict,
                }
            return sample
        elif self.task_name == "refcoco":
            patch_image_size = self.cfg.task.patch_image_size
            w, h = image.size
            w_resize_ratio = torch.tensor(patch_image_size / w).unsqueeze(0)
            h_resize_ratio = torch.tensor(patch_image_size / h).unsqueeze(0)
            patch_image = self.patch_resize_transform(image).unsqueeze(0)
            patch_mask = torch.tensor([True])
            src_text = self.encode_text(' which region does the text " {} " describe?'.format(text), append_bos=True,
                                   append_eos=True).unsqueeze(0)
            src_length = torch.LongTensor([s.ne(self.pad_idx).long().sum() for s in src_text])
            sample = {
                "id": np.array(['42']),
                "net_input": {
                    "src_tokens": src_text,
                    "src_lengths": src_length,
                    "patch_images": patch_image,
                    "patch_masks": patch_mask,
                },
                "w_resize_ratios": w_resize_ratio,
                "h_resize_ratios": h_resize_ratio,
                "region_coords": torch.randn(1, 4)
            }
            return sample

    def infer(self, img, text):
        # get cv2 image
        if self.task_name == "caption" or self.task_name == "vqa_gen":
            image = cv2.resize(img, dsize=(640, 480)) # NOTE forcely
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(image)
            # Construct input sample & preprocess for GPU if cuda available
            sample = self.construct_sample(image, text)
            sample = utils.move_to_cuda(sample) if self.use_cuda else sample
            sample = utils.apply_to_sample(apply_half, sample) if self.use_fp16 else sample
            if self.task_name == "caption":
                with torch.no_grad():
                    result, scores = eval_step(self.task, self.generator, self.models, sample)
                text = result[0]['caption']
                return text
            elif self.task_name == "vqa_gen":
                with torch.no_grad():
                    result, scores = zero_shot_step(self.task, self.generator, self.models, sample)
                text = result[0]['answer']
                return text
        elif self.task_name == "refcoco":
            pass

# run
if __name__ == "__main__":
    app = Flask(__name__)
    ofa_task = os.environ["OFA_TASK"]
    ofa_model_scale = os.environ["OFA_MODEL_SCALE"]
    # TODO add refcoco
    # if ofa_task == "all":
    caption_infer = Inference("caption", ofa_model_scale)
    vqa_infer = Inference("vqa_gen", ofa_model_scale)

    # elif ofa_task == "caption":
    #     caption_infer = Inference("caption", ofa_model_scale)

    # elif ofa_task == "vqa_gen":
    #     vqa_infer = Inference("vqa_gen", ofa_model_scale)


    @app.route("/caption", methods=['POST'])
    def caption_request():
        data = request.data.decode("utf-8")
        data_json = json.loads(data)
        # process image
        image_b = data_json['image']
        image_dec = base64.b64decode(image_b)
        data_np = np.fromstring(image_dec, dtype='uint8')
        img = cv2.imdecode(data_np, 1)
        # get text
        texts = data_json['queries']
        results = []
        for text in texts:
            answer = caption_infer.infer(img, text)
            results.append({"question": text, "answer": answer})
        return Response(response=json.dumps({"results": results}), status=200)

    @app.route("/vqa_gen", methods=['POST'])
    def vqa_request():
        data = request.data.decode("utf-8")
        data_json = json.loads(data)
        # process image
        image_b = data_json['image']
        image_dec = base64.b64decode(image_b)
        data_np = np.fromstring(image_dec, dtype='uint8')
        img = cv2.imdecode(data_np, 1)
        # get text
        texts = data_json['queries']
        results = []
        for text in texts:
            answer = vqa_infer.infer(img, text)
            results.append({"question": text, "answer": answer})
        return Response(response=json.dumps({"results": results}), status=200)
    
    app.run("0.0.0.0", 8080, threaded=True)
