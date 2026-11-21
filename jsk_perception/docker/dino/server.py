from groundingdino.util.inference import load_model, load_image, predict, annotate
import groundingdino.datasets.transforms as T
from torchvision.ops import box_convert

import cv2
import numpy as np
from PIL import Image as PLImage
import torch

# web server
from flask import Flask, request, Response
import json
import base64


def apply_half(t):
    if t.dtype is torch.float32:
        return t.to(dtype=torch.half)
    return t

class Inference:
    def __init__(self, gpu_id=None):
        self.gpu_id = gpu_id
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = load_model("groundingdino/config/GroundingDINO_SwinT_OGC.py", "weights/groundingdino_swint_ogc.pth")
        self.BOX_TRESHOLD = 0.35
        self.TEXT_TRESHOLD = 0.25

    def convert_to_string(self, input_list):
        output_string = ""
        for item in input_list:
            output_string += item + " . "
        return output_string.strip()

    def infer(self, img, texts):
        # get cv2 image
        # image = cv2.resize(img, dsize=(640, 480)) # NOTE forcely
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image_source = PLImage.fromarray(image)
        image = np.asarray(image_source)
        transform = T.Compose(
            [
                T.RandomResize([800], max_size=1333),
                T.ToTensor(),
                T.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ]
        )
        image_transformed, _ = transform(image_source, None)

        image_source = image
        image = image_transformed

        TEXT_PROMPT = self.convert_to_string(texts)

        boxes, logits, phrases = predict(
            model=self.model,
            image=image,
            caption=TEXT_PROMPT,
            box_threshold=self.BOX_TRESHOLD,
            text_threshold=self.TEXT_TRESHOLD,
            device = self.device
        )

        h, w, _ = image_source.shape
        boxes = boxes * torch.Tensor([w, h, w, h])
        xyxy = box_convert(boxes=boxes, in_fmt="cxcywh", out_fmt="xyxy").numpy()

        results = {}
        for i in range(len(xyxy)):
            box = xyxy[i].tolist()
            logit = logits[i].item()
            results[i] = {"box": box, "logit": logit, "phrase": phrases[i]}

        return results

# run
if __name__ == "__main__":
    app = Flask(__name__)
    infer = Inference()

    @app.route("/detection", methods=['POST'])
    def detection_request():
        data = request.data.decode("utf-8")
        data_json = json.loads(data)
        # process image
        image_b = data_json['image']
        image_dec = base64.b64decode(image_b)
        data_np = np.fromstring(image_dec, dtype='uint8')
        img = cv2.imdecode(data_np, 1)
        # get text
        texts = data_json['queries']
        infer_results = infer.infer(img, texts)
        results = []
        for i in range(len(infer_results)):
            results.append({"id": i, "box": infer_results[i]["box"], "logit": infer_results[i]["logit"], "phrase": infer_results[i]["phrase"]})
        return Response(response=json.dumps({"results": results}), status=200)

    app.run("0.0.0.0", 8080, threaded=True)
