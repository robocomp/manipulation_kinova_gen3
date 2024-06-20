from PIL import Image
from ultralytics import YOLO

class YoloDetector:
    def __init__(self):
        self.model = YOLO("yolov8n.pt")

    def detect(self, source):
        results = self.model(source)
        return results

    def plot(self, results):
        for i, r in enumerate(results):
            im_bgr = r.plot()
            im_rgb = Image.fromarray(im_bgr[..., ::-1])
            r.show()
            r.save(filename=f"results{i}.jpg")