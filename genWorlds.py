from lxml import etree
import random
import math
import copy
import argparse
from collections import namedtuple

class Tree:
  def __init__(self, id_in, pose_in, scale_in, mesh_num_in):
    self.id = 'tree' + str(id_in)
    self.pose = pose_in
    self.scale = scale_in
    self.mesh_num = mesh_num_in

class GenXML:
  def __init__(self):
    self.root = etree.Element('world',name='default')

    #disable shadows
    scene_el= etree.Element('scene')
    shadows_el = etree.Element('shadows')
    shadows_el.text = 'false'
    scene_el.append(shadows_el)
    self.root.append(scene_el)

    #create the forest base model
    model_el = etree.Element('model', name = 'forest')
    static_el = etree.Element('static')
    static_el.text = 'true'
    model_el.append(static_el)
    self.root.append(model_el)

    #plugins
    self.add_plugins()

    #physics
    self.add_physics()

    #done
    self.add_coordinate_system()

    # in progress
    self.add_ground_plane()

    self.add_sun()

    #self.add_void()

    #self.add_user_camera()





  def add_model(self, name_in, pose_in, scale_in, mesh_num_in, models_type):

    link_el = etree.Element('link', name = name_in)
    static_el = etree.Element('static')
    static_el.text = '1'
    pose_el = etree.Element('pose')
    pose_el.text = ''.join(str(e) + ' ' for e in pose_in)
    link_el.append(pose_el)

    visual_el = etree.Element('visual', name = 'visual')
    geometry_el = etree.Element('geometry')
    cylinder_el = etree.SubElement(geometry_el,'cylinder')
    radius_el = etree.SubElement(cylinder_el,'radius')
    radius_el.text = str(scale_in)
    length_el = etree.SubElement(cylinder_el,'length')
    length_el.text = '10'
    material_el = etree.SubElement(visual_el,'material')
    script_el = etree.SubElement(material_el,'script')
    uri1_el = etree.SubElement(script_el,'uri')
    uri1_el.text = 'model://tree_simple/scripts'
    uri2_el = etree.SubElement(script_el,'uri')
    uri2_el.text = 'model://tree_simple/materials/textures'

    
    visual_el.append(geometry_el)
    link_el.append(visual_el)

    collision_el = etree.Element('collision', name = 'collision')
    collision_el.append(copy.deepcopy(geometry_el))
    contacts_el = etree.Element('max_contacts')
    contacts_el.text = '0'
    collision_el.append(contacts_el)
    link_el.append(collision_el)

    self.root.find('model').append(link_el)

  def add_plugins(self):
    #start plugin section
    plugins_comment_el = etree.Comment(' Plugins ')
    self.root.append(plugins_comment_el)

    mrs_transform_plug_el = etree.Element('plugin', name = 'mrs_gazebo_link_attacher_plugin', 
                                          filename = 'libMrsGazeboCommonResources_StaticTransformRepublisher.so')
    self.root.append(mrs_transform_plug_el)

    mrs_rviz_plug_el = etree.Element('plugin', name = 'mrs_gazebo_rviz_cam_synchronizer', 
                                          filename = 'libMrsGazeboCommonResources_RvizCameraSynchronizer.so')

    self.root.append(mrs_rviz_plug_el)

  def add_physics(self):
    #start physics section
    physics_comment_el = etree.Comment(' Physics ')
    self.root.append(physics_comment_el)

    #physics
    physics_el = etree.Element('physics', name = 'default_physics',
                               default = '0', type = 'ode')
    max_step_size_el = etree.SubElement(physics_el, 'max_step_size')
    max_step_size_el.text = '0.004'
    real_time_factor_el = etree.SubElement(physics_el, 'real_time_factor')
    real_time_factor_el.text = '1'
    real_time_update_rate_el = etree.SubElement(physics_el,
                                                'real_time_update_rate')
    real_time_update_rate_el.text = '250'
    gravity_el = etree.SubElement(physics_el, 'gravity')
    gravity_el.text = '0 0 -9.8066'
    magnetic_field_el = etree.SubElement(physics_el, 'magnetic_field')
    magnetic_field_el.text = '6e-06 2.3e-05 -4.2e-05'

    #ode
    ode_el = etree.SubElement(physics_el, 'ode')
    #solver
    solver_el = etree.SubElement(ode_el, 'solver')
    type_el = etree.SubElement(solver_el, 'type')
    type_el.text = 'quick'
    iters_el = etree.SubElement(solver_el, 'iters')
    iters_el.text = '10'
    sor_el = etree.SubElement(solver_el, 'sor')
    sor_el.text = '1.3'
    use_dynamic_moi_rescaling = etree.SubElement(solver_el,
                                                 'use_dynamic_moi_rescaling')
    use_dynamic_moi_rescaling.text = '0'
    #constraints
    constraints_el = etree.SubElement(ode_el, 'constraints')
    cfm_el = etree.SubElement(constraints_el, 'cfm')
    cfm_el.text = '0'
    erp_el = etree.SubElement(constraints_el, 'erp')
    erp_el.text = '0.2'
    contact_max_correcting_vel_el = etree.SubElement(constraints_el,
                                                  'contact_max_correcting_vel')
    contact_max_correcting_vel_el.text = '1000'
    contact_surface_layer_el = etree.SubElement(constraints_el,
                                                  'contact_surface_layer')
    contact_surface_layer_el.text = '0.001'
    self.root.append(physics_el)

  def add_coordinate_system(self):
    #start section comoment
    coordinate_comment_el = etree.Comment(' Coordinate System ')
    self.root.append(coordinate_comment_el)

    # tree
    coordinates_el = etree.Element('spherical_coordinates')
    surface_model_el = etree.SubElement(coordinates_el, 'surface_model')
    surface_model_el.text = 'EARTH_WGS84'
    latitude_deg_el = etree.SubElement(coordinates_el, 'latitude_deg')
    latitude_deg_el.text = '47.397743'
    longitude_deg_el = etree.SubElement(coordinates_el,'longitude_deg')
    longitude_deg_el.text = '8.545594'
    elevation_el = etree.SubElement(coordinates_el, 'elevation')
    elevation_el.text = '0.0'
    heading_deg_el = etree.SubElement(coordinates_el, 'heading_deg')
    heading_deg_el.text = '0'
    
    self.root.append(coordinates_el)

  def add_ground_plane(self):
    #start  section
    ground_comment_el = etree.Comment(' Ground Plane ')
    self.root.append(ground_comment_el)

    #tree
    ground_el = etree.Element('model', name="ground_plane")
    static_el = etree.SubElement(ground_el, 'static')
    static_el.text = 'true'

    link_el = etree.SubElement(ground_el, 'link', name="link")

    collision_el = etree.SubElement(link_el, 'collision', name="collision")
    
    pose_el = etree.SubElement(collision_el, 'pose')
    pose_el.text = '0 0 0 0 0 0'

    geometry_el = etree.SubElement(collision_el, 'geometry')
    plane_el = etree.SubElement(geometry_el, 'plane')
    normal_el = etree.SubElement(plane_el, 'normal')
    normal_el.text = '0 0 1'
    size_el = etree.SubElement(normal_el, 'size')
    size_el.text = '250 250'


    surface_el = etree.SubElement(collision_el, 'surface')
    friction_el = etree.SubElement(surface_el, 'friction')
    ode_el = etree.SubElement(friction_el, 'ode')
    mu_el = etree.SubElement(ode_el, 'mu')
    mu_el.text = '1'
    mu2_el = etree.SubElement(ode_el, 'mu2')
    mu2_el.text = '1'

    visual_el = etree.SubElement(link_el, 'visual', name="grass")
    pose_el2 = etree.SubElement(visual_el, 'pose')
    pose_el2.text = '0 0 0 0 0 0'
    cast_shadows_el = etree.SubElement(visual_el, 'cast_shadows')
    cast_shadows_el.text = 'false'
    geometry_el2 = etree.SubElement(visual_el, 'geometry')
    mesh_el = etree.SubElement(geometry_el2, 'mesh')
    uri_el = etree.SubElement(mesh_el, 'uri')
    uri_el.text = '/opt/ros/noetic/share/mrs_gazebo_common_resources/models/grass_plane/meshes/grass_plane.dae'

    self.root.append(ground_el)

  def add_sun(self):
    #start  section
    sun_comment_el = etree.Comment(' Sun ')
    self.root.append(sun_comment_el)

    #tree
    sun_el = etree.Element('light', name="sun", type='directional')
    pose_el = etree.SubElement(sun_el, 'pose', frame='')
    pose_el.text = '0 0 1000 0.4 0.2 0'

    specular_el = etree.SubElement(sun_el, 'specular')
    specular_el.text = '0.6 0.6 0.6 1'

    direction_el = etree.SubElement(sun_el, 'direction')
    direction_el.text = '0.1 0.1 -0.9'
    attenuation_el = etree.SubElement(sun_el, 'attenuation')
    range_el = etree.SubElement(attenuation_el, 'range')
    range_el.text = '20'
    constant_el = etree.SubElement(attenuation_el, 'constant')
    constant_el.text = '0.5'
    linear_el = etree.SubElement(attenuation_el, 'linear')
    linear_el.text = '0.01'
    quadratic_el = etree.SubElement(attenuation_el, 'quadratic')
    quadratic_el.text = '0.001'
    cast_shadows_el = etree.SubElement(sun_el, 'cast_shadows')
    cast_shadows_el.text = '1'

    self.root.append(sun_el)

  def output_xml(self):
    sdf = etree.Element('sdf',version='1.4')
    sdf.append(self.root)

    return '<?xml version="1.0"?>\n' + etree.tostring(sdf, pretty_print=True).decode()

class World:
  TOTAL_NUM_MESHES = 1

  def __init__(self, world_length, use_high_res):
    self.world_length = world_length
    self.trees = []

    if(use_high_res):
      self.models_type = 'models_high_res'
    else:
      self.models_type = 'models_low_res'

  def _gen_random_tree(self):
    id_num = len(self.trees)
    x = random.uniform(-self.world_length/2, self.world_length/2)
    y = random.uniform(-self.world_length/2, self.world_length/2)
    
    x_nono_min = -self.world_length * 0.1
    x_nono_max = self.world_length * 0.1
    y_nono_min = -self.world_length * 0.1
    y_nono_max = self.world_length * 0.1
    
    while x >= x_nono_min and x <= x_nono_max and y >= y_nono_min and y <= y_nono_max:
      x = random.uniform(-self.world_length/2, self.world_length/2)
      y = random.uniform(-self.world_length/2, self.world_length/2)
    
    angle = random.uniform(0, 2*math.pi)
    scale = random.uniform(0.1, 0.2)
    mesh_num = random.randint(1, self.TOTAL_NUM_MESHES)

    return Tree(id_num, [x,y,0,0,0,angle], scale, mesh_num)

  def add_trees(self, num_trees):
    for i in range(num_trees):
      self.trees.append(self._gen_random_tree())

  def save_world(self, filename):

    xml = GenXML()
    for tree in self.trees:
      xml.add_model(tree.id,
                    tree.pose,
                    tree.scale,
                    tree.mesh_num,
                    self.models_type)

    text_file = open(filename, "w")
    text_file.write(xml.output_xml())
    text_file.close()

def gen_worlds(save_path, num_worlds, world_length, num_trees, use_high_res):
  for i in range(num_worlds):
    world = World(world_length, use_high_res)
    world.add_trees(num_trees)
    world.save_world(save_path + '/forest' + str(i) + '.world')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate a random gazebo forest.')
    parser.add_argument('--num_worlds', type=int, help='Number of worlds to generate')
    parser.add_argument('--world_length', type=int, help='Length and width of world in m')
    parser.add_argument('--tree_density', type=float, help='Number of trees per m^2')
    parser.add_argument('--high_res', type=int, help='Use high res tree models')
    args = parser.parse_args()

    gen_worlds('/home/chris/catkin_ws/src/forest_gen/worlds', args.num_worlds, args.world_length,
               int(args.world_length*args.world_length*args.tree_density),
               bool(args.high_res))
