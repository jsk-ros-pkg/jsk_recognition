import chainer
import chainer.functions as F
import chainer.links as L


class EncoderFC3Dropout(chainer.Chain):

    def __init__(self):
        super(EncoderFC3Dropout, self).__init__()
        with self.init_scope():
            self.fc1 = L.Linear(2133, 1024)
            self.fc2 = L.Linear(1024, 1024)
            self.fc3 = L.Linear(1024, 85)

    def __call__(self, x):
        h = self.fc1(x)
        h = F.relu(h)
        h = F.dropout(h)
        h = self.fc2(h)
        h = F.relu(h)
        h = F.dropout(h)
        h = self.fc3(h)
        return h
