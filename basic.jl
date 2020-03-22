mutable struct Revolute
  parent 
  child
  origin
  name
  function Revolute(parent, child, origin, name)
    new(parent, child, origin, name)
  end
end

mutable struct Link
  parent
  child
  name
  length
  color
  function Link(name; length=1.0, color=:red)
    new(Nothing, Nothing, name, length, color)
  end
end

struct Transform
  trans
  rot
end

function excert(tf::Transform, vec)
  return tf.trans + Rot2d(tf.rot)*vec
end


mutable struct Robot
  links 
  joints
  tfs 
  q 
  function Robot(link_list, joint_list)
    joints = Dict()

    links = Dict()
    for fr in link_list
      links[fr.name] = fr
    end

    println(links)
    for jt in joint_list
      joints[jt.name] = jt
      links[jt.parent].child = jt
      links[jt.child].parent = jt
    end

    tfs = Nothing
    q = Nothing
    new(links, joints, tfs, q)
  end
end

function Rot2d(theta)
  m = [cos(theta) -sin(theta);
       sin(theta) cos(theta)]
end

function set_configuration(r::Robot, q)
  @assert length(q) == length(collect(r.joints))
  # precompute minimal transformation 
  r.tfs = []
  tfs = Dict()
  for pair in zip(values(r.joints), q)
    jt, angle = pair 
    key = (jt.child, jt.parent)
    print(key)
    println(angle)
    tf = Transform(jt.origin[1:2], angle)
    #tf(vec) = Rot2d(angle)*vec[1:2] + jt.origin[1:2]
    tfs[key] = tf
  end

  for link in values(r.links)
    tfs[(link.name, link.name)] = Transform([0., 0.], 0.0)
  end

  r.tfs = tfs
  r.q = q
end

function get_parent_link(r::Robot, link::Link)
  link.parent == Nothing && (return Nothing)
  joint_parent = r.joints[link.parent.name]
  link_parent = r.links[joint_parent.parent]
end

function get_child_link(r::Robot, link::Link)
  # currently each link has a single child
  link.child == Nothing && (return Nothing)
  joint_child = r.joints[link.child.name]
  link_child = r.links[joint_child.child]
end

function composite_tf(tf_a2b::Transform, tf_b2c::Transform)
  trans_new = tf_b2c.trans + Rot2d(tf_b2c.rot)*tf_a2b.trans
  rot_new = tf_b2c.rot + tf_a2b.rot
  tf_a2c = Transform(trans_new, rot_new)
  return tf_a2c
end

function get_tf(r::Robot, link_child, link_parent)
  # razy evaluation of get_tf
  # computation is done recursively
  key_try = (link_child.name, link_parent.name)
  print(key_try)
  try
    tf = r.tfs[key_try]
    return tf

  catch error 
    if isa(error, KeyError) # key not found
      link_middle = get_parent_link(r, link_child)
      tf_c2m = get_tf(r, link_child, link_middle) # must exist
      tf_m2p = get_tf(r, link_middle, link_parent) # into a recursion
      tf_c2p = composite_tf(tf_c2m, tf_m2p)

      r.tfs[key_try] = tf_c2p
      return tf_c2p
    end
  end
end

function jacobian(r::Robot, link)
  vec_d_list = []
  for joint in values(r.joints)
    coord_here = r.links[joint.parent]
    tf = get_tf(r, link, coord_here)
    vec_r = excert(tf, [0, 0])
    vec_vertical = [-vec_r[2], vec_r[1]]
    println(vec_vertical)
    tf_to_world = get_tf(r, coord_here, r.links["world"])
    vec_d = excert(tf_to_world, vec_vertical)
    push!(vec_d_list, vec_d)
  end
  jac = hcat(vec_d_list...)
  return jac
end

function visualize(r::Robot)
  points = []
  points_tip = []
  plot()
  for link in values(r.links)
    tf = get_tf(r, link.name, "world")
    point = tf([0, 0])
    point_tip = tf([link.length, 0])  
    push!(points, point)
    if link.name != "world"
      plot!([point[1], point_tip[1]], [point[2], point_tip[2]], legend=false, color=link.color)
    end
  end
  #return points
  arr = hcat(points...)
  plot!(arr[1, :], arr[2, :], seriestype = :scatter, legend=false)
end

link_world = Link("world")
link_body1 = Link("body1")
link_body2 = Link("body2")
link_body3 = Link("body3")
link_body4 = Link("body4"; length=0.5, color=:blue)

joint1 = Revolute("world", "body1", [0, 0, 0], "joint1")
joint2 = Revolute("body1", "body2", [1, 0, 0], "joint2")
joint3 = Revolute("body2", "body3", [1, 0, 0], "joint3")
joint4 = Revolute("body3", "body4", [1, 0, 0], "joint4")

link_list = [link_world, link_body1, link_body2, link_body3, link_body4]
joint_list = [joint1, joint2, joint3, joint4]

r = Robot(link_list, joint_list)
get_tf(r, link_body4, link_body1)

@time set_configuration(r, [0.2, -0.2, 0.2, 0.3])
j = jacobian(r, link_body3)
