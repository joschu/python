#!/usr/bin/env python

import numpy as np
import lfd
from lfd import recognition, warping, registration

def draw_comparison(left_data, right_data, corr_data):
  from traits.api import HasTraits, Instance, Button, on_trait_change
  from traitsui.api import View, Item, HSplit, Group
  from mayavi import mlab
  from mayavi.core.api import Engine
  from mayavi.core.ui.api import MlabSceneModel, SceneEditor, MayaviScene

  class ComparisonDialog(HasTraits):
    engine1 = Instance(Engine, ())
    scene1 = Instance(MlabSceneModel, ())
    engine2 = Instance(Engine, ())
    scene2 = Instance(MlabSceneModel, ())
    view = View(
      HSplit(
        Group(
          Item(
            'scene1',
            editor=SceneEditor(scene_class=MayaviScene),
            height=400,
            width=400,
            show_label=False,
          ),
          show_labels=False,
        ),
        Group(
          Item(
            'scene2',
            editor=SceneEditor(scene_class=MayaviScene),
            height=400,
            width=400,
            show_label=False
          ),
          show_labels=False,
        ),
      ),
      resizable=True,
    )

    def _scene1_default(self):
      self.engine1.start()
      return MlabSceneModel(engine=self.engine1)

    def _scene2_default(self):
      self.engine2.start()
      return MlabSceneModel(engine=self.engine2)


#   def draw_shape_context(self, scene, xyz_matched):
#     assert 'shape_context_costs' in xyz_matched
#     costs, matched_ds = xyz_matched['shape_context_costs'].copy(), xyz_matched['cloud_xyz_ds']
#     costs -= min(costs)
#     #costs /= max(costs) + 1
#     costs /= 0.5
#     print costs, costs.shape
#     print matched_ds.shape
#     x, y, z = matched_ds.transpose()
#     mlab.points3d(x, y, z, costs, mode='sphere', figure=scene.mayavi_scene)

    def draw_source_cloud(self, scene, data):
      def xyz_to_coords(xyz):
        if xyz.shape[1] == 2:
          x, y = xyz.transpose()
          z = np.zeros(x.shape)
        elif xyz.shape[1] == 3:
          x, y, z = xyz.transpose()
        else:
          raise NotImplementedError
        return x, y, z
      cloud = xyz_to_coords(data['xyz'])
      cloud_ds = xyz_to_coords(data['xyz_ds'])
      data['glyphs'] = mlab.points3d(cloud[0], cloud[1], cloud[2], mode='point', figure=scene.mayavi_scene)
      data['glyphs_ds'] = mlab.points3d(cloud_ds[0], cloud_ds[1], cloud_ds[2], mode='sphere', scale_factor=0.008, figure=scene.mayavi_scene)

    def setup_picker(self, fig, glyphs, callback):
      def cb_wrapper(picker):
        glyph_points = glyphs.glyph.glyph_source.glyph_source.output.points.to_array()
        if picker.actor in glyphs.actor.actors:
          point_id = picker.point_id/glyph_points.shape[0]
          if point_id != -1:
            callback(point_id)
      fig.on_mouse_pick(cb_wrapper)

    @on_trait_change('scene1.activated')
    def redraw_scene1(self):
      mlab.clf(figure=self.scene1.mayavi_scene)
      self.draw_source_cloud(self.scene1, left_data)

      self.left_pick = mlab.outline(line_width=3, color=(1, 1, 0), name='left_picker', figure=self.scene1.mayavi_scene)
      self.left_pick.outline_mode = 'cornered'
      self.left_pick.bounds = (0, 0, 0, 0, 0, 0)

      def pick_cb(point_id):
        x, y, z = left_data['xyz_ds'][point_id]
        self.left_pick.bounds = (x-0.01, x+0.01,
                                 y-0.01, y+0.01,
                                 z-0.01, z+0.01)

        x, y, z = right_data['xyz_ds'][corr_data['partners'][point_id]]
        self.right_pick.bounds = (x-0.01, x+0.01,
                                  y-0.01, y+0.01,
                                  z-0.01, z+0.01)

      self.setup_picker(self.scene1.mayavi_scene, left_data['glyphs_ds'], pick_cb)


    @on_trait_change('scene2.activated')
    def redraw_scene2(self):
      mlab.clf(figure=self.scene2.mayavi_scene)
      self.draw_source_cloud(self.scene2, right_data)
      self.right_pick = mlab.outline(line_width=3, color=(1, 0, 1), name='right_picker', figure=self.scene2.mayavi_scene)
      self.right_pick.outline_mode = 'cornered'
      self.right_pick.bounds = (0, 0, 0, 0, 0, 0)

#     pick_pt_outline = mlab.outline(line_width=3, color=(1, 0, 1), name='right_picker', figure=self.scene2.mayavi_scene)
#     pick_pt_outline.outline_mode = 'cornered'
#     pick_pt_outline.bounds = (0, 0, 0, 0, 0, 0)

#     pick_pt_outline1 = mlab.outline(line_width=3, color=(1, 1, 0), name='left_picker', figure=self.scene1.mayavi_scene)
#     pick_pt_outline1.outline_mode = 'cornered'
#     pick_pt_outline1.bounds = (0, 0, 0, 0, 0, 0)

#     def pick_cb(point_id):
#       # show point picked on right
#       x, y, z = right_data['xyz_ds'][point_id]
#       pick_pt_outline.bounds = (x-0.01, x+0.01,
#                                 y-0.01, y+0.01,
#                                 z-0.01, z+0.01)
#       # show corresponding point on left
#       x, y, z = left_data['xyz_ds'][corr_data['partners'][point_id]]
#       pick_pt_outline1.bounds = (x-0.01, x+0.01,
#                                  y-0.01, y+0.01,
#                                  z-0.01, z+0.01)

#     self.setup_picker(self.scene2.mayavi_scene, right_data['glyphs_ds'], pick_cb)

  m = ComparisonDialog()
  m.configure_traits()

def main():
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('clouds', nargs='+', help='cloud resource srings')
  args = parser.parse_args()

  assert len(args.clouds) == 2

  import cloud_io
  print 'loading clouds'
  clouds = [cloud_io.read_from_rsrc(c) for c in args.clouds]
  print 'processing'
  clouds_ds = [recognition.downsample(c)[0] for c in clouds]
#  hists_ds = [recognition.calc_shape_context_hists(dc) for dc in clouds_ds]
  f, partners, hists0, hists1, costs = recognition.match_and_calc_shape_context(clouds_ds[0], clouds_ds[1], return_tuple=True)

  left_data = {
    'xyz': clouds[0],
    'xyz_ds': clouds_ds[0],
    'hists_ds': hists0
  }
  right_data = {
    'xyz': clouds[1],
    'xyz_ds': clouds_ds[1],
    'hists_ds': hists1
  }
  corr_data = {
    'f': f,
    'partners': partners,
    'costs': costs
  }

  print 'drawing'
  draw_comparison(left_data, right_data, corr_data)

if __name__ == '__main__':
  main()
