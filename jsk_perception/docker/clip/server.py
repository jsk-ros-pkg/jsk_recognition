import clip
import cv2
import numpy as np
import os
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
        self.model, self.preprocess = clip.load('ViT-B/32', self.device)

    def infer(self, img, texts):
        # get cv2 image
        image = cv2.resize(img, dsize=(640, 480)) # NOTE forcely
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PLImage.fromarray(image)
        image_input = self.preprocess(image).unsqueeze(0).to(self.device)
        text_inputs = torch.cat([clip.tokenize(c) for c in texts]).to(self.device)
        with torch.no_grad():
            image_features = self.model.encode_image(image_input)
            text_features = self.model.encode_text(text_inputs)
        image_features /= image_features.norm(dim=-1, keepdim=True)
        text_features /= text_features.norm(dim=-1, keepdim=True)
        probability = (100.0 * image_features @ text_features.T).softmax(dim=-1)
        similarity = (text_features.cpu().numpy() @ image_features.cpu().numpy().T).T[0]  # cosine similarity
        values, indices = probability[0].topk(len(texts))
        results = {}
        for value, index in zip(values, indices):
            results[texts[index]] = (value.item(), float(similarity[index]))
        return results

# run
if __name__ == "__main__":
    app = Flask(__name__)
    infer = Inference()

    @app.route("/inference", methods=['POST'])
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
        infer_results = infer.infer(img, texts)
        results = []
        for q in infer_results:
            results.append({"question": q, "probability": infer_results[q][0], "similarity": infer_results[q][1]})
        return Response(response=json.dumps({"results": results}), status=200)
    
    app.run("0.0.0.0", 8080, threaded=True)
