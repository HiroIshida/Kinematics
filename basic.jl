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
  function Link(name)
    new(Nothing, Nothing, name)
  end
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
    tf(vec) = Rot2d(angle)*vec[1:2] + jt.origin[1:2]
    tfs[key] = tf
  end
  r.tfs = tfs
  r.q = q
end

function get_parent_link(r::Robot, link::Link)
  joint_parent = r.joints[link.parent.name]
  link_parent = r.links[joint_parent.parent]
end

function get_tf(r::Robot, link_desc_name, link_asc_name)
  key_try = (link_desc_name, link_asc_name)
  try
    tf = r.tfs[key_try]
    return tf
  catch error
    if isa(error, KeyError) # key not found
      link_asc = r.links[link_asc_name]
      link_desc = r.links[link_desc_name]

      function recursion(vec, link)
        link == link_asc && (return vec)

        link_parent = get_parent_link(r, link)
        name_this = link.name
        name_parent = link_parent.name

        key = (name_this, name_parent)
        tf_single = r.tfs[key] #lambda
        recursion(tf_single(vec), link_parent)
      end
      tf(vec) = recursion(vec, link_desc)
      r.tfs[(link_desc_name, link_asc_name)] = tf
      return tf
    end
  end
end

function visualize(r::Robot)
  points = []
  points_tip = []
  for link in values(r.links)
    tf = get_tf(r, link.name, "world")
    point = tf([0, 0])
    point_tip = tf([1, 0])
    push!(points, point)
    if link.name != "world"
      plot!([point[1], point_tip[1]], [point[2], point_tip[2]], legend=false)
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
link_body4 = Link("body4")

joint1 = Revolute("world", "body1", [0, 0, 0], "joint1")
joint2 = Revolute("body1", "body2", [1, 0, 0], "joint2")
joint3 = Revolute("body2", "body3", [1, 0, 0], "joint3")
joint4 = Revolute("body3", "body4", [1, 0, 0], "joint4")

link_list = [link_world, link_body1, link_body2, link_body3, link_body4]
joint_list = [joint1, joint2, joint3, joint4]

r = Robot(link_list, joint_list)

tf = get_tf(r, "body1", "world") 
#tf([1, 0])

@time set_configuration(r, [0.2, -0.2, 0.2, 0.3])
using Plots
visualize(r)
