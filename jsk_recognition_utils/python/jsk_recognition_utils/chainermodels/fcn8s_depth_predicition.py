import chainer
from chainer import cuda
import chainer.functions as F
import chainer.links as L
import fcn
import numpy as np


class FCN8sDepthPrediction(chainer.Chain):

    # [0.2, 3]
    min_depth = 0.2
    max_depth = 3.

    def __init__(self, n_class, masking=True, concat=True):
        self.n_class = n_class
        self.masking = masking
        self.concat = concat
        kwargs = {
            'initialW': chainer.initializers.Zero(),
            'initial_bias': chainer.initializers.Zero(),
        }
        super(self.__class__, self).__init__()
        with self.init_scope():
            self.conv_rgb_1_1 = L.Convolution2D(3, 64, 3, 1, 100, **kwargs)
            self.conv_rgb_1_2 = L.Convolution2D(64, 64, 3, 1, 1, **kwargs)

            self.conv_rgb_2_1 = L.Convolution2D(64, 128, 3, 1, 1, **kwargs)
            self.conv_rgb_2_2 = L.Convolution2D(128, 128, 3, 1, 1, **kwargs)

            self.conv_rgb_3_1 = L.Convolution2D(128, 256, 3, 1, 1, **kwargs)
            self.conv_rgb_3_2 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)
            self.conv_rgb_3_3 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)

            self.conv_rgb_4_1 = L.Convolution2D(256, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_4_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_4_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.conv_rgb_5_1 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_5_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_5_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.rgb_fc6 = L.Convolution2D(512, 4096, 7, 1, 0, **kwargs)
            self.rgb_fc7 = L.Convolution2D(4096, 4096, 1, 1, 0, **kwargs)

            self.mask_score_fr = L.Convolution2D(
                4096, n_class, 1, 1, 0, **kwargs)

            self.mask_upscore2 = L.Deconvolution2D(
                n_class, n_class, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())
            self.mask_upscore8 = L.Deconvolution2D(
                n_class, n_class, 16, 8, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

            self.mask_score_pool3 = L.Convolution2D(
                256, n_class, 1, 1, 0, **kwargs)
            self.mask_score_pool4 = L.Convolution2D(
                512, n_class, 1, 1, 0, **kwargs)

            self.mask_upscore_pool4 = L.Deconvolution2D(
                n_class, n_class, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

            self.conv_depth_1_1 = L.Convolution2D(3, 64, 3, 1, 100, **kwargs)
            self.conv_depth_1_2 = L.Convolution2D(64, 64, 3, 1, 1, **kwargs)

            self.conv_depth_2_1 = L.Convolution2D(64, 128, 3, 1, 1, **kwargs)
            self.conv_depth_2_2 = L.Convolution2D(128, 128, 3, 1, 1, **kwargs)

            self.conv_depth_3_1 = L.Convolution2D(128, 256, 3, 1, 1, **kwargs)
            self.conv_depth_3_2 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)
            self.conv_depth_3_3 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)

            self.conv_depth_4_1 = L.Convolution2D(256, 512, 3, 1, 1, **kwargs)
            self.conv_depth_4_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_depth_4_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.conv_depth_5_1 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_depth_5_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_depth_5_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            if self.concat is True:
                self.concat_fc6 = L.Convolution2D(
                    1024, 4096, 7, 1, 0, **kwargs)
            else:
                self.depth_fc6 = L.Convolution2D(512, 4096, 7, 1, 0, **kwargs)

            self.concat_fc7 = L.Convolution2D(4096, 4096, 1, 1, 0, **kwargs)

            self.depth_score_fr = L.Convolution2D(4096, 1, 1, 1, 0, **kwargs)

            self.depth_upscore2 = L.Deconvolution2D(
                1, 1, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())
            self.depth_upscore8 = L.Deconvolution2D(
                1, 1, 16, 8, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

            self.depth_score_pool3 = L.Convolution2D(
                256, 1, 1, 1, 0, **kwargs)
            self.depth_score_pool4 = L.Convolution2D(
                512, 1, 1, 1, 0, **kwargs)

            self.depth_upscore_pool4 = L.Deconvolution2D(
                1, 1, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

    def predict_mask(self, rgb, return_pool5=False):
        # conv_rgb_1
        h = F.relu(self.conv_rgb_1_1(rgb))
        h = F.relu(self.conv_rgb_1_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool1 = h  # 1/2

        # conv_rgb_2
        h = F.relu(self.conv_rgb_2_1(rgb_pool1))
        h = F.relu(self.conv_rgb_2_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool2 = h  # 1/4

        # conv_rgb_3
        h = F.relu(self.conv_rgb_3_1(rgb_pool2))
        h = F.relu(self.conv_rgb_3_2(h))
        h = F.relu(self.conv_rgb_3_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool3 = h  # 1/8

        # conv_rgb_4
        h = F.relu(self.conv_rgb_4_1(rgb_pool3))
        h = F.relu(self.conv_rgb_4_2(h))
        h = F.relu(self.conv_rgb_4_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool4 = h  # 1/16

        # conv_rgb_5
        h = F.relu(self.conv_rgb_5_1(rgb_pool4))
        h = F.relu(self.conv_rgb_5_2(h))
        h = F.relu(self.conv_rgb_5_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        rgb_pool5 = h  # 1/32

        # rgb_fc6
        h = F.relu(self.rgb_fc6(rgb_pool5))
        h = F.dropout(h, ratio=.5)
        rgb_fc6 = h  # 1/32

        # rgb_fc7
        h = F.relu(self.rgb_fc7(rgb_fc6))
        h = F.dropout(h, ratio=.5)
        rgb_fc7 = h  # 1/32

        # mask_score_fr
        h = self.mask_score_fr(rgb_fc7)
        mask_score_fr = h  # 1/32

        # mask_score_pool3
        scale_rgb_pool3 = 0.0001 * rgb_pool3
        h = self.mask_score_pool3(scale_rgb_pool3)
        mask_score_pool3 = h  # 1/8

        # mask_score_pool4
        scale_rgb_pool4 = 0.01 * rgb_pool4
        h = self.mask_score_pool4(scale_rgb_pool4)
        mask_score_pool4 = h  # 1/16

        # mask upscore2
        h = self.mask_upscore2(mask_score_fr)
        mask_upscore2 = h  # 1/16

        # mask_score_pool4c
        h = mask_score_pool4[:, :,
                             5:5 + mask_upscore2.data.shape[2],
                             5:5 + mask_upscore2.data.shape[3]]
        mask_score_pool4c = h  # 1/16

        # mask_fuse_pool4
        h = mask_upscore2 + mask_score_pool4c
        mask_fuse_pool4 = h  # 1/16

        # mask_upscore_pool4
        h = self.mask_upscore_pool4(mask_fuse_pool4)
        mask_upscore_pool4 = h  # 1/8

        # mask_score_pool3c
        h = mask_score_pool3[:, :,
                             9:9 + mask_upscore_pool4.data.shape[2],
                             9:9 + mask_upscore_pool4.data.shape[3]]
        mask_score_pool3c = h  # 1/8

        # mask_fuse_pool3
        h = mask_upscore_pool4 + mask_score_pool3c
        mask_fuse_pool3 = h  # 1/8

        # mask_upscore8
        h = self.mask_upscore8(mask_fuse_pool3)
        mask_upscore8 = h  # 1/1

        # mask_score
        h = mask_upscore8[:, :,
                          31:31 + rgb.shape[2], 31:31 + rgb.shape[3]]
        mask_score = h  # 1/1

        if return_pool5:
            return mask_score, rgb_pool5
        else:
            return mask_score

    def predict_depth(self, rgb, mask_score, depth_viz, rgb_pool5):
        # conv_depth_1
        h = F.relu(self.conv_depth_1_1(depth_viz))
        h = F.relu(self.conv_depth_1_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool1 = h  # 1/2

        # conv_depth_2
        h = F.relu(self.conv_depth_2_1(depth_pool1))
        h = F.relu(self.conv_depth_2_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool2 = h  # 1/4

        # conv_depth_3
        h = F.relu(self.conv_depth_3_1(depth_pool2))
        h = F.relu(self.conv_depth_3_2(h))
        h = F.relu(self.conv_depth_3_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool3 = h  # 1/8

        # conv_depth_4
        h = F.relu(self.conv_depth_4_1(depth_pool3))
        h = F.relu(self.conv_depth_4_2(h))
        h = F.relu(self.conv_depth_4_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool4 = h  # 1/16

        # conv_depth_5
        h = F.relu(self.conv_depth_5_1(depth_pool4))
        h = F.relu(self.conv_depth_5_2(h))
        h = F.relu(self.conv_depth_5_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool5 = h  # 1/32

        if self.masking is True:
            # Apply negative_mask to depth_pool5
            # (N, C, H, W) -> (N, H, W)
            mask_pred_tmp = F.argmax(self.mask_score, axis=1)
            # (N, H, W) -> (N, 1, H, W), float required for resizing
            mask_pred_tmp = mask_pred_tmp[:, None, :, :].data.astype(
                self.xp.float32)  # 1/1
            resized_mask_pred = F.resize_images(
                mask_pred_tmp,
                (depth_pool5.shape[2], depth_pool5.shape[3]))  # 1/32
            depth_pool5_cp = depth_pool5
            masked_depth_pool5 = depth_pool5_cp * \
                (resized_mask_pred.data == 0.0).astype(self.xp.float32)
        else:
            masked_depth_pool5 = depth_pool5

        if self.concat is True:
            # concatenate rgb_pool5 and depth_pool5
            concat_pool5 = F.concat((rgb_pool5, masked_depth_pool5), axis=1)

            # concat_fc6
            h = F.relu(self.concat_fc6(concat_pool5))
            h = F.dropout(h, ratio=.5)
            concat_fc6 = h  # 1/32
        else:
            # concat_fc6
            h = F.relu(self.depth_fc6(masked_depth_pool5))
            h = F.dropout(h, ratio=.5)
            concat_fc6 = h  # 1/32

        # concat_fc7
        h = F.relu(self.concat_fc7(concat_fc6))
        h = F.dropout(h, ratio=.5)
        concat_fc7 = h  # 1/32

        # depth_score_fr
        h = self.depth_score_fr(concat_fc7)
        depth_score_fr = h  # 1/32

        # depth_score_pool3
        scale_depth_pool3 = 0.0001 * depth_pool3
        h = self.depth_score_pool3(scale_depth_pool3)
        depth_score_pool3 = h  # 1/8

        # depth_score_pool4
        scale_depth_pool4 = 0.01 * depth_pool4
        h = self.depth_score_pool4(scale_depth_pool4)
        depth_score_pool4 = h  # 1/16

        # depth upscore2
        h = self.depth_upscore2(depth_score_fr)
        depth_upscore2 = h  # 1/16

        # depth_score_pool4c
        h = depth_score_pool4[:, :,
                              5:5 + depth_upscore2.data.shape[2],
                              5:5 + depth_upscore2.data.shape[3]]
        depth_score_pool4c = h  # 1/16

        # depth_fuse_pool4
        h = depth_upscore2 + depth_score_pool4c
        depth_fuse_pool4 = h  # 1/16

        # depth_upscore_pool4
        h = self.depth_upscore_pool4(depth_fuse_pool4)
        depth_upscore_pool4 = h  # 1/8

        # depth_score_pool3c
        h = depth_score_pool3[:, :,
                              9:9 + depth_upscore_pool4.data.shape[2],
                              9:9 + depth_upscore_pool4.data.shape[3]]
        depth_score_pool3c = h  # 1/8

        # depth_fuse_pool3
        h = depth_upscore_pool4 + depth_score_pool3c
        depth_fuse_pool3 = h  # 1/8

        # depth_upscore8
        h = self.depth_upscore8(depth_fuse_pool3)
        depth_upscore8 = h  # 1/1

        # depth_score
        h = depth_upscore8[:, :,
                           31:31 + rgb.shape[2],
                           31:31 + rgb.shape[3]]
        depth_score = h  # 1/1

        return depth_score

    def compute_loss_mask(self, mask_score, true_mask):
        # segmentation loss
        seg_loss = F.softmax_cross_entropy(
            mask_score, true_mask, normalize=True)

        return seg_loss

    def compute_loss_depth(self, depth_pred, true_mask, true_depth):
        assert true_mask.dtype == self.xp.int32

        # Whole region
        keep_regardless_mask = ~self.xp.isnan(true_depth)
        if self.xp.sum(keep_regardless_mask) == 0:
            depth_loss_regardless_mask = 0
        else:
            depth_loss_regardless_mask = F.sum(F.log(F.cosh(
                depth_pred[keep_regardless_mask[self.xp.newaxis, :, :, :]] -
                true_depth[keep_regardless_mask])))
            depth_loss_regardless_mask /= \
                true_depth.shape[1] * true_depth.shape[2]

        # Only masked region
        keep_only_mask = self.xp.logical_and(
            true_mask > 0, ~self.xp.isnan(true_depth))
        if self.xp.sum(keep_only_mask) == 0:
            depth_loss_only_mask = 0
        else:
            depth_loss_only_mask = F.sum(F.log(F.cosh(
                depth_pred[keep_only_mask[self.xp.newaxis, :, :, :]] -
                true_depth[keep_only_mask])))
            depth_loss_only_mask /= true_depth.shape[1] + true_depth.shape[2]

        # Regression loss
        coef = [1, 1]
        reg_loss = (coef[0] * depth_loss_regardless_mask +
                    coef[1] * depth_loss_only_mask)

        return reg_loss

    def compute_loss(self, mask_score, depth_pred, true_mask, true_depth):
        seg_loss = self.compute_loss_mask(mask_score, true_mask)
        reg_loss = self.compute_loss_depth(depth_pred, true_mask, true_depth)

        # Loss
        coef = [1, 1]
        loss = coef[0] * seg_loss + coef[1] * reg_loss
        if self.xp.isnan(float(loss.data)):
            raise ValueError('Loss is nan.')

        batch_size = len(mask_score)
        assert batch_size == 1

        # GPU -> CPU
        # N, C, H, W -> C, H, W
        true_mask = cuda.to_cpu(true_mask)[0]
        mask_pred = cuda.to_cpu(F.argmax(self.score_label, axis=1).data)[0]
        true_depth = cuda.to_cpu(true_depth)[0]
        depth_pred = cuda.to_cpu(depth_pred.data)[0]

        # Evaluate Mask IU
        mask_iu = fcn.utils.label_accuracy_score(
            [true_mask], [mask_pred], n_class=2)[2]

        # Evaluate Depth Accuracy
        depth_acc = {}
        for thresh in [0.01, 0.02, 0.03, 0.04, 0.05, 0.07, 0.10, 0.15, 0.20,
                       0.25, 0.30, 0.40, 0.50, 0.70, 1.00]:
            t_lbl_fg = true_mask > 0
            p_lbl_fg = mask_pred > 0
            if np.sum(t_lbl_fg) == 0 and np.sum(p_lbl_fg) == 0:
                acc = 1.0
            elif np.sum(t_lbl_fg) == 0:
                acc = 0.0
            else:
                # {TP and (|error| < thresh)} / (TP or FP or FN)
                true_depth_cp = np.copy(true_depth)
                true_depth_cp[np.isnan(true_depth_cp)] = np.inf
                numer = np.sum(
                    np.logical_and(
                        np.logical_and(t_lbl_fg, p_lbl_fg),
                        np.abs(true_depth_cp - depth_pred) < thresh))
                denom = np.sum(np.logical_or(t_lbl_fg, p_lbl_fg))
                acc = 1. * numer / denom
            depth_acc['%.2f' % thresh] = acc

        chainer.reporter.report({
            'loss': loss,
            'seg_loss': seg_loss,
            'reg_loss': reg_loss,
            'miou': mask_iu,
            'depth_acc<0.01': depth_acc['0.01'],
            'depth_acc<0.02': depth_acc['0.02'],
            'depth_acc<0.03': depth_acc['0.03'],
            'depth_acc<0.04': depth_acc['0.04'],
            'depth_acc<0.05': depth_acc['0.05'],
            'depth_acc<0.07': depth_acc['0.07'],
            'depth_acc<0.10': depth_acc['0.10'],
            'depth_acc<0.15': depth_acc['0.15'],
            'depth_acc<0.20': depth_acc['0.20'],
            'depth_acc<0.25': depth_acc['0.25'],
            'depth_acc<0.30': depth_acc['0.30'],
            'depth_acc<0.40': depth_acc['0.40'],
            'depth_acc<0.50': depth_acc['0.50'],
            'depth_acc<0.70': depth_acc['0.70'],
            'depth_acc<1.00': depth_acc['1.00'],
        }, self)

        return loss

    def __call__(self, rgb, depth_viz):
        mask_score, rgb_pool5 = self.predict_mask(rgb, return_pool5=True)
        self.mask_score = mask_score

        depth_score = self.predict_depth(
            rgb, mask_score, depth_viz, rgb_pool5)
        self.depth_score = depth_score

        assert not chainer.config.train
        return

    def init_from_vgg16(self, vgg16):
        for l in self.children():
            if l.name.startswith('conv'):
                l1 = getattr(vgg16,
                             l.name.split('_')[0] + l.name.split('_')[2] +
                             '_' + l.name.split('_')[3])
                l2 = getattr(self, l.name)
                assert l1.W.shape == l2.W.shape
                assert l1.b.shape == l2.b.shape
                l2.W.data[...] = l1.W.data[...]
                l2.b.data[...] = l1.b.data[...]
            elif l.name in ['rgb_fc6', 'rgb_fc7']:
                l1 = getattr(vgg16, l.name.split('_')[1])
                l2 = getattr(self, l.name)
                assert l1.W.size == l2.W.size
                assert l1.b.size == l2.b.size
                l2.W.data[...] = l1.W.data.reshape(l2.W.shape)[...]
                l2.b.data[...] = l1.b.data.reshape(l2.b.shape)[...]
            elif l.name == 'depth_fc6' and self.concat is False:
                l1 = getattr(vgg16, 'fc6')
                l2 = getattr(self, l.name)
                assert l1.W.size == l2.W.size
                assert l1.b.size == l2.b.size
                l2.W.data[...] = l1.W.data.reshape(l2.W.shape)[...]
                l2.b.data[...] = l1.b.data.reshape(l2.b.shape)[...]
            elif l.name == 'concat_fc6' and self.concat is True:
                l1 = getattr(vgg16, 'fc6')
                l2 = getattr(self, l.name)
                assert l1.W.size * 2 == l2.W.size
                assert l1.b.size == l2.b.size
                l2.W.data[:, :int(l2.W.shape[1] / 2), :, :] = \
                    l1.W.data.reshape(
                        (l2.W.shape[0], int(l2.W.shape[1] / 2),
                         l2.W.shape[2], l2.W.shape[3]))[...]
                l2.W.data[:, int(l2.W.shape[1] / 2):, :, :] = \
                    l1.W.data.reshape(
                        (l2.W.shape[0], int(l2.W.shape[1] / 2),
                         l2.W.shape[2], l2.W.shape[3]))[...]
                l2.b.data[...] = l1.b.data.reshape(l2.b.shape)[...]
            elif l.name == 'concat_fc7':
                l1 = getattr(vgg16, 'fc7')
                l2 = getattr(self, l.name)
                assert l1.W.size == l2.W.size
                assert l1.b.size == l2.b.size
                l2.W.data[...] = l1.W.data.reshape(l2.W.shape)[...]
                l2.b.data[...] = l1.b.data.reshape(l2.b.shape)[...]


class FCN8sDepthPredictionConcatFirst(FCN8sDepthPrediction):

    # [0.4, 5.1]
    min_depth = 0.4
    max_depth = 5.1

    def __init__(self, n_class, masking=True):
        self.n_class = n_class
        self.masking = masking
        kwargs = {
            'initialW': chainer.initializers.Zero(),
            'initial_bias': chainer.initializers.Zero(),
        }
        super(FCN8sDepthPrediction, self).__init__()  # This is correct
        with self.init_scope():
            self.conv_rgb_1_1 = L.Convolution2D(3, 64, 3, 1, 100, **kwargs)
            self.conv_rgb_1_2 = L.Convolution2D(64, 64, 3, 1, 1, **kwargs)

            self.conv_rgb_2_1 = L.Convolution2D(64, 128, 3, 1, 1, **kwargs)
            self.conv_rgb_2_2 = L.Convolution2D(128, 128, 3, 1, 1, **kwargs)

            self.conv_rgb_3_1 = L.Convolution2D(128, 256, 3, 1, 1, **kwargs)
            self.conv_rgb_3_2 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)
            self.conv_rgb_3_3 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)

            self.conv_rgb_4_1 = L.Convolution2D(256, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_4_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_4_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.conv_rgb_5_1 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_5_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_rgb_5_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.rgb_fc6 = L.Convolution2D(512, 4096, 7, 1, 0, **kwargs)
            self.rgb_fc7 = L.Convolution2D(4096, 4096, 1, 1, 0, **kwargs)

            self.mask_score_fr = L.Convolution2D(
                4096, n_class, 1, 1, 0, **kwargs)

            self.mask_upscore2 = L.Deconvolution2D(
                n_class, n_class, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())
            self.mask_upscore8 = L.Deconvolution2D(
                n_class, n_class, 16, 8, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

            self.mask_score_pool3 = L.Convolution2D(
                256, n_class, 1, 1, 0, **kwargs)
            self.mask_score_pool4 = L.Convolution2D(
                512, n_class, 1, 1, 0, **kwargs)

            self.mask_upscore_pool4 = L.Deconvolution2D(
                n_class, n_class, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

            self.conv_depth_1_1 = L.Convolution2D(6, 64, 3, 1, 100, **kwargs)
            self.conv_depth_1_2 = L.Convolution2D(64, 64, 3, 1, 1, **kwargs)

            self.conv_depth_2_1 = L.Convolution2D(64, 128, 3, 1, 1, **kwargs)
            self.conv_depth_2_2 = L.Convolution2D(128, 128, 3, 1, 1, **kwargs)

            self.conv_depth_3_1 = L.Convolution2D(128, 256, 3, 1, 1, **kwargs)
            self.conv_depth_3_2 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)
            self.conv_depth_3_3 = L.Convolution2D(256, 256, 3, 1, 1, **kwargs)

            self.conv_depth_4_1 = L.Convolution2D(256, 512, 3, 1, 1, **kwargs)
            self.conv_depth_4_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_depth_4_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.conv_depth_5_1 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_depth_5_2 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)
            self.conv_depth_5_3 = L.Convolution2D(512, 512, 3, 1, 1, **kwargs)

            self.depth_fc6 = L.Convolution2D(512, 4096, 7, 1, 0, **kwargs)
            self.depth_fc7 = L.Convolution2D(4096, 4096, 1, 1, 0, **kwargs)

            self.depth_score_fr = L.Convolution2D(4096, 1, 1, 1, 0, **kwargs)

            self.depth_upscore2 = L.Deconvolution2D(
                1, 1, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())
            self.depth_upscore8 = L.Deconvolution2D(
                1, 1, 16, 8, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

            self.depth_score_pool3 = L.Convolution2D(
                256, 1, 1, 1, 0, **kwargs)
            self.depth_score_pool4 = L.Convolution2D(
                512, 1, 1, 1, 0, **kwargs)

            self.depth_upscore_pool4 = L.Deconvolution2D(
                1, 1, 4, 2, 0, nobias=True,
                initialW=fcn.initializers.UpsamplingDeconvWeight())

    def predict_depth(self, rgb, mask_score, depth_viz):
        # concatenate rgb and depth_viz
        concat_input = F.concat((rgb, depth_viz), axis=1)

        # conv_depth_1
        h = F.relu(self.conv_depth_1_1(concat_input))
        h = F.relu(self.conv_depth_1_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool1 = h  # 1/2

        # conv_depth_2
        h = F.relu(self.conv_depth_2_1(depth_pool1))
        h = F.relu(self.conv_depth_2_2(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool2 = h  # 1/4

        # conv_depth_3
        h = F.relu(self.conv_depth_3_1(depth_pool2))
        h = F.relu(self.conv_depth_3_2(h))
        h = F.relu(self.conv_depth_3_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool3 = h  # 1/8

        # conv_depth_4
        h = F.relu(self.conv_depth_4_1(depth_pool3))
        h = F.relu(self.conv_depth_4_2(h))
        h = F.relu(self.conv_depth_4_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool4 = h  # 1/16

        # conv_depth_5
        h = F.relu(self.conv_depth_5_1(depth_pool4))
        h = F.relu(self.conv_depth_5_2(h))
        h = F.relu(self.conv_depth_5_3(h))
        h = F.max_pooling_2d(h, 2, stride=2, pad=0)
        depth_pool5 = h  # 1/32

        if self.masking is True:
            # (N, C, H, W) -> (N, H, W)
            mask_pred_tmp = F.argmax(self.score_label, axis=1)
            # (N, H, W) -> (N, 1, H, W), float required for resizing
            mask_pred_tmp = mask_pred_tmp[:, None, :, :].data.astype(
                self.xp.float32)  # 1/1
            resized_mask_pred = F.resize_images(
                mask_pred_tmp,
                (depth_pool5.shape[2], depth_pool5.shape[3]))  # 1/32
            depth_pool5_cp = depth_pool5
            masked_depth_pool5 = depth_pool5_cp * \
                (resized_mask_pred.data == 0.0).astype(self.xp.float32)
        else:
            masked_depth_pool5 = depth_pool5

        # depth_fc6
        h = F.relu(self.depth_fc6(masked_depth_pool5))
        h = F.dropout(h, ratio=.5)
        depth_fc6 = h  # 1/32

        # depth_fc7
        h = F.relu(self.depth_fc7(depth_fc6))
        h = F.dropout(h, ratio=.5)
        depth_fc7 = h  # 1/32

        # depth_score_fr
        h = self.depth_score_fr(depth_fc7)
        depth_score_fr = h  # 1/32

        # depth_score_pool3
        scale_depth_pool3 = 0.0001 * depth_pool3
        h = self.depth_score_pool3(scale_depth_pool3)
        depth_score_pool3 = h  # 1/8

        # depth_score_pool4
        scale_depth_pool4 = 0.01 * depth_pool4
        h = self.depth_score_pool4(scale_depth_pool4)
        depth_score_pool4 = h  # 1/16

        # depth upscore2
        h = self.depth_upscore2(depth_score_fr)
        depth_upscore2 = h  # 1/16

        # depth_score_pool4c
        h = depth_score_pool4[:, :,
                              5:5 + depth_upscore2.data.shape[2],
                              5:5 + depth_upscore2.data.shape[3]]
        depth_score_pool4c = h  # 1/16

        # depth_fuse_pool4
        h = depth_upscore2 + depth_score_pool4c
        depth_fuse_pool4 = h  # 1/16

        # depth_upscore_pool4
        h = self.depth_upscore_pool4(depth_fuse_pool4)
        depth_upscore_pool4 = h  # 1/8

        # depth_score_pool3c
        h = depth_score_pool3[:, :,
                              9:9 + depth_upscore_pool4.data.shape[2],
                              9:9 + depth_upscore_pool4.data.shape[3]]
        depth_score_pool3c = h  # 1/8

        # depth_fuse_pool3
        h = depth_upscore_pool4 + depth_score_pool3c
        depth_fuse_pool3 = h  # 1/8

        # depth_upscore8
        h = self.depth_upscore8(depth_fuse_pool3)
        depth_upscore8 = h  # 1/1

        # depth_score
        h = depth_upscore8[:, :,
                           31:31 + rgb.shape[2],
                           31:31 + rgb.shape[3]]
        depth_score = h  # 1/1

        # (-inf, inf) -> (0, 1) -> (min_depth, max_depth)
        h = F.sigmoid(depth_score)
        h = h * (self.max_depth - self.min_depth) + self.min_depth
        depth_pred = h

        return depth_pred

    def __call__(self, rgb, depth_viz, mask_gt=None, depth_gt=None):
        score_label = self.predict_mask(rgb, return_pool5=False)
        self.score_label = score_label
        depth_pred = self.predict_depth(rgb, score_label, depth_viz)
        self.depth_score = depth_pred

        if mask_gt is None or depth_gt is None:
            assert not chainer.config.train
            return

        loss = self.compute_loss(
            score_label, depth_pred, mask_gt, depth_gt)

        return loss

    def init_from_vgg16(self, vgg16):
        for l in self.children():
            if l.name == 'conv_depth_1_1':
                l1 = getattr(vgg16, 'conv1_1')
                l2 = getattr(self, l.name)
                assert l1.W.size * 2 == l2.W.size
                assert l1.b.size == l2.b.size
                l2.W.data[:, :int(l2.W.shape[1] / 2), :, :] = \
                    l1.W.data.reshape(
                        (l2.W.shape[0], int(l2.W.shape[1] / 2),
                         l2.W.shape[2], l2.W.shape[3]))[...]
                l2.W.data[:, int(l2.W.shape[1] / 2):, :, :] = \
                    l1.W.data.reshape(
                        (l2.W.shape[0], int(l2.W.shape[1] / 2),
                         l2.W.shape[2], l2.W.shape[3]))[...]
                l2.b.data[...] = l1.b.data.reshape(l2.b.shape)[...]
            elif l.name.startswith('conv'):
                l1 = getattr(vgg16,
                             l.name.split('_')[0] + l.name.split('_')[2] +
                             '_' + l.name.split('_')[3])
                l2 = getattr(self, l.name)
                assert l1.W.shape == l2.W.shape
                assert l1.b.shape == l2.b.shape
                l2.W.data[...] = l1.W.data[...]
                l2.b.data[...] = l1.b.data[...]
            elif l.name in ['rgb_fc6', 'rgb_fc7']:
                l1 = getattr(vgg16, l.name.split('_')[1])
                l2 = getattr(self, l.name)
                assert l1.W.size == l2.W.size
                assert l1.b.size == l2.b.size
                l2.W.data[...] = l1.W.data.reshape(l2.W.shape)[...]
                l2.b.data[...] = l1.b.data.reshape(l2.b.shape)[...]
            elif l.name in ['depth_fc6', 'depth_fc7']:
                l1 = getattr(vgg16, l.name.split('_')[1])
                l2 = getattr(self, l.name)
                assert l1.W.size == l2.W.size
                assert l1.b.size == l2.b.size
                l2.W.data[...] = l1.W.data.reshape(l2.W.shape)[...]
                l2.b.data[...] = l1.b.data.reshape(l2.b.shape)[...]
