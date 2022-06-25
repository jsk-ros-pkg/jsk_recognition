from __future__ import absolute_import

import torch
import numpy as np

from .import_dr_spaam import dr_spaam
from dr_spaam.model.drow_net import DrowNet
from dr_spaam.model.dr_spaam import DrSpaam
from dr_spaam.utils import utils as u


class DRSpaamDetector(object):

    def __init__(
        self, ckpt_file, model="DROW3", gpu=-1, stride=1, panoramic_scan=False
    ):
        """A warpper class around DROW3 or DR-SPAAM network for end-to-end inference.

        Args:
            ckpt_file (str): Path to checkpoint
            model (str): Model name, "DROW3" or "DR-SPAAM".
            gpu (Int): If greater equal than 0, use gpu.
            stride (int): Downsample scans for faster inference.
            panoramic_scan (bool): True if the scan covers 360 degree.
        """
        if gpu >= 0:
            torch.backends.cudnn.benchmark = True
            self.device = torch.device('cuda:{}'.format(gpu))
        else:
            self.device = torch.device('cpu')
        self._stride = stride
        self._use_dr_spaam = model == "DR-SPAAM"

        self._scan_phi = None
        self._laser_fov_deg = None

        if model == "DROW3":
            self._model = DrowNet(
                dropout=0.5, cls_loss=None, mixup_alpha=0.0, mixup_w=0.0
            )
        elif model == "DR-SPAAM":
            self._model = DrSpaam(
                dropout=0.5,
                num_pts=56,
                embedding_length=128,
                alpha=0.5,
                window_size=17,
                panoramic_scan=panoramic_scan,
                cls_loss=None,
                mixup_alpha=0.0,
                mixup_w=0.0,
            )
        else:
            raise NotImplementedError(
                "model should be 'DROW3' or 'DR-SPAAM', received {} instead.".format(
                    model
                )
            )

        ckpt = torch.load(ckpt_file, map_location=torch.device('cpu'))
        self._model.load_state_dict(ckpt["model_state"])

        self._model.eval()
        self._model = self._model.to(self.device)

    def __call__(self, scan):
        if self._scan_phi is None:
            assert self.is_ready(), "Call set_laser_fov() first."
            half_fov_rad = 0.5 * np.deg2rad(self._laser_fov_deg)
            self._scan_phi = np.linspace(
                -half_fov_rad, half_fov_rad, len(scan), dtype=np.float32
            )

        # preprocess
        ct = u.scans_to_cutout(
            scan[None, ...],
            self._scan_phi,
            stride=self._stride,
            centered=True,
            fixed=True,
            window_width=1.0,
            window_depth=0.5,
            num_cutout_pts=56,
            padding_val=29.99,
            area_mode=True,
        )
        ct = torch.from_numpy(ct).float()
        ct = ct.to(self.device)

        # inference
        sim = None
        with torch.no_grad():
            # one extra dimension for batch
            if self._use_dr_spaam:
                pred_cls, pred_reg, sim = self._model(ct.unsqueeze(dim=0), inference=True)
            else:
                pred_cls, pred_reg = self._model(ct.unsqueeze(dim=0))
        if sim is not None:
            sim = sim.data.cpu().numpy()

        pred_cls = torch.sigmoid(pred_cls[0]).data.cpu().numpy()
        pred_reg = pred_reg[0].data.cpu().numpy()

        # postprocess
        dets_xy, dets_cls, instance_mask = u.nms_predicted_center(
            scan[:: self._stride],
            self._scan_phi[:: self._stride],
            pred_cls[:, 0],
            pred_reg,
        )

        return dets_xy, dets_cls, instance_mask, sim

    def set_laser_fov(self, fov_deg):
        self._laser_fov_deg = fov_deg

    def is_ready(self):
        return self._laser_fov_deg is not None
