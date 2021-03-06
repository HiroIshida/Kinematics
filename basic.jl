using LinearAlgebra

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

function excert(tf::Transform, vec; rotonly=false)
  if rotonly
    return Rot2d(tf.rot)*vec
  else
    return tf.trans + Rot2d(tf.rot)*vec
  end
end


mutable struct Robot
  links 
  joints
  tfs 
  q 
  qd
  function Robot(link_list, joint_list)
    joints = Dict()

    links = Dict()
    for fr in link_list
      links[fr.name] = fr
    end

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

function set_configuration(r::Robot, q, qd = nothing)
  @assert length(q) == length(collect(r.joints))
  # precompute minimal transformation 
  r.tfs = []
  tfs = Dict()
  for pair in zip(values(r.joints), q)
    jt, angle = pair 
    key = (jt.child, jt.parent)
    tf = Transform(jt.origin[1:2], angle)
    #tf(vec) = Rot2d(angle)*vec[1:2] + jt.origin[1:2]
    tfs[key] = tf
  end

  for link in values(r.links)
    tfs[(link.name, link.name)] = Transform([0., 0.], 0.0)
  end

  r.tfs = tfs
  r.q = q
  r.qd = qd
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
    coord_here = r.links[joint.child]
    tf = get_tf(r, link, coord_here)
    vec_r = excert(tf, [0, 0])
    vec_vertical = [-vec_r[2], vec_r[1]]
    tf_to_world = get_tf(r, coord_here, r.links["world"])
    vec_d = excert(tf_to_world, vec_vertical; rotonly=true)
    push!(vec_d_list, vec_d)
  end
  jac = hcat(vec_d_list...)
  return jac
end

function visualize(r::Robot)
  points = []
  points_tip = []
  plot()
  link_root = r.links["world"]
  for link in values(r.links)
    tf = get_tf(r, link, link_root)
    point = excert(tf, [0, 0])
    point_tip = excert(tf, [link.length, 0])  
    push!(points, point)
    if link.name != "world"
      plot!([point[1], point_tip[1]], [point[2], point_tip[2]], legend=false, color=link.color)
    end
  end
  #return points
  arr = hcat(points...)
  plot!(arr[1, :], arr[2, :], seriestype = :scatter, legend=false)
end

function sr_inverse(j)
  n = size(j)[1]
  j_inv = j'*inv(j * j' + Diagonal(ones(n)))
end

function solve_ik!(r::Robot, x_target; itr=100)
  for i in 1:itr
    tf = get_tf(r, link_body4, r.links["world"])
    x_now = tf.trans
    #println(x_now)
    dx = x_target - x_now
    j = jacobian(r, link_body3)[:, 1:end-1]
    j_inv = sr_inverse(j)
    if i==1
      print(j_inv)
    end
    dq = j_inv * dx
    push!(dq, 0)
    set_configuration(r, r.q + dq, r.qd)
  end
end

link_world = Link("world")
link_body1 = Link("body1")
link_body2 = Link("body2")
link_body3 = Link("body3")
link_body4 = Link("body4"; length=0.1, color=:blue)

joint1 = Revolute("world", "body1", [0, 0, 0], "joint1")
joint2 = Revolute("body1", "body2", [1, 0, 0], "joint2")
joint3 = Revolute("body2", "body3", [1, 0, 0], "joint3")
joint4 = Revolute("body3", "body4", [1, 0, 0], "joint4")

link_list = [link_world, link_body1, link_body2, link_body3, link_body4]
joint_list = [joint1, joint2, joint3, joint4]

r = Robot(link_list, joint_list)
#get_tf(r, link_body4, link_body1)

set_configuration(r, zeros(4), zeros(4))
#j = jacobian(r, link_body3)
#x_target = [1., 0.5]
#@time solve_ik!(r, x_target)

#@time solve_ik!(r, x_target)

function simple_attractor(x, xd) 
  x_g = [1., 0.5]
  #x_g = [1.9, 0.9]
  alpha = 2.0
  beta = 10.0

  function normalizer(s)  
    c = 0.01
    z = norm()
    h = z + c * log(1+exp(-2*c*z))
    return s/h
  end

  print(beta * xd)
  f = alpha * (x_g - x) - beta * xd
  return f
end

x = [0, 0]
xd = [0, 0]
dt = 0.1

for i in 1:500
  j = jacobian(r, link_body4)
  xd = j*r.qd

  tf = get_tf(r, link_body4, r.links["world"])
  x = tf.trans
  xdd = simple_attractor(x, xd)
  println(xdd)

  j_ = j[:, 1:end-1]
  qdd = sr_inverse(j_'*j_)*j_'*xdd
  push!(qdd, 0.0)

  qd = r.qd + qdd * dt
  q = r.q + r.qd * dt
  set_configuration(r, q, qd)
  #visualize(r)
end

using Plots
#plot()
visualize(r)

