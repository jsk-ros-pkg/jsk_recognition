from imagebind import data
from imagebind.models import imagebind_model
from imagebind.models.imagebind_model import ModalityType
from pytorchvideo.data.encoded_video_decord import EncodedVideoDecord
import io

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
    def __init__(self, modal, gpu_id=None):
        self.gpu_id = gpu_id
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        self.modal_name = modal

        self.model = imagebind_model.imagebind_huge(pretrained=True)
        self.model.eval()
        self.model.to(self.device)

        self.video_sample_rate=16000

    def convert_to_string(self, input_list):
        output_string = ""
        for item in input_list:
            output_string += item + " . "
        return output_string.strip()

    def infer(self, msg, texts):
        text_inputs = texts

        if self.modal_name == "image":
            # get cv2 image
            # image = cv2.resize(img, dsize=(640, 480)) # NOTE forcely
            # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.cvtColor(msg, cv2.COLOR_BGR2RGB)
            image = PLImage.fromarray(image)

            image_input = [image]

            inputs = {
                ModalityType.TEXT: data.load_and_transform_text(text_inputs, self.device),
                ModalityType.VISION: data.load_and_transform_vision_data(None, self.device, image_input),
            }
            modal_data_type = ModalityType.VISION

        elif self.modal_name == "video":
            import decord
            decord.bridge.set_bridge("torch")
            video_io = io.BytesIO(msg)
            video = EncodedVideoDecord(file=video_io,
                                       video_name="current_video_data",
                                       decode_video=True,
                                       decode_audio=False,
                                       **{"sample_rate": self.video_sample_rate},
            )

            inputs = {
                ModalityType.TEXT: data.load_and_transform_text(text_inputs, self.device),
                ModalityType.VISION: data.load_and_transform_video_data(None, self.device, videos=[video]),
            }
            modal_data_type = ModalityType.VISION

        elif self.modal_name == "audio":
            waveform = msg["waveform"]
            sr = msg["sr"]
            waveform_np = np.frombuffer(waveform, dtype=np.float32)
            waveform_torch = torch.tensor(waveform_np.reshape(1, -1))

            inputs = {
                ModalityType.TEXT: data.load_and_transform_text(text_inputs, self.device),
                ModalityType.AUDIO: data.load_and_transform_audio_data(None, self.device, audios=[{"waveform": waveform_torch, "sr": sr}]),
            }
            modal_data_type = ModalityType.AUDIO

        # Calculate features
        with torch.no_grad():
            embeddings = self.model(inputs)

        similarity = np.average((embeddings[modal_data_type] @ embeddings[ModalityType.TEXT].T).tolist(), axis=0)
        probability = torch.softmax(embeddings[modal_data_type] @ embeddings[ModalityType.TEXT].T, dim=-1)

        values, indices = probability[0].topk(len(texts))
        results = {}
        for value, index in zip(values, indices):
            results[texts[index]] = (value.item(), float(similarity[index]))
        return results

# run
if __name__ == "__main__":
    app = Flask(__name__)

    image_infer = Inference("image")
    video_infer = Inference("video")
    audio_infer = Inference("audio")

    try:
        @app.route("/inference", methods=['POST'])
        def image_request():
            data = request.data.decode("utf-8")
            data_json = json.loads(data)
            # process image
            image_b = data_json['image']
            image_dec = base64.b64decode(image_b)
            data_np = np.fromstring(image_dec, dtype='uint8')
            img = cv2.imdecode(data_np, 1)
            # get text
            texts = data_json['queries']
            infer_results = image_infer.infer(img, texts)
            results = []
            for q in infer_results:
                results.append({"question": q, "probability": infer_results[q][0], "similarity": infer_results[q][1]})
            return Response(response=json.dumps({"results": results}), status=200)
    except NameError:
        print("Skipping create inference app")

    try:
        @app.route("/video_class", methods=['POST'])
        def video_request():
            data = request.data.decode("utf-8")
            data_json = json.loads(data)
            # process image
            video_b = data_json['video']
            video_dec = base64.b64decode(video_b)
            # get text
            texts = data_json['queries']
            infer_results = video_infer.infer(video_dec, texts)
            results = []
            for q in infer_results:
                results.append({"question": q, "probability": infer_results[q][0], "similarity": infer_results[q][1]})
            return Response(response=json.dumps({"results": results}), status=200)
    except NameError:
        print("Skipping create video_class app")

    try:
        @app.route("/audio_class", methods=['POST'])
        def audio_request():
            data = request.data.decode("utf-8")
            data_json = json.loads(data)
            # process image
            audio_b = data_json['audio']
            sr = data_json['sr']
            audio_dec = base64.b64decode(audio_b)
            # get text
            texts = data_json['queries']
            infer_results = audio_infer.infer({"waveform": audio_dec, "sr": sr}, texts)
            results = []
            for q in infer_results:
                results.append({"question": q, "probability": infer_results[q][0], "similarity": infer_results[q][1]})
            return Response(response=json.dumps({"results": results}), status=200)
    except NameError:
        print("Skipping create audio_class app")

    app.run("0.0.0.0", 8080, threaded=True)
