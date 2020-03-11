mutable struct Revolute
  parent 
  child
  origin
  name
  function Revolute(parent, child, origin, name)
    new(parent, child, origin, name)
  end
end

mutable struct Frame
  parent
  child
  name
  function Frame(name)
    new(Nothing, Nothing, name)
  end
end


mutable struct Robot
  frames 
  joints
  tfs 
  q 
  function Robot(frame_list, joint_list)
    joints = Dict()

    frames = Dict()
    for fr in frame_list
      frames[fr.name] = fr
    end

    println(frames)
    for jt in joint_list
      joints[jt.name] = jt
      frames[jt.parent].child = jt
      frames[jt.child].parent = jt
    end

    tfs = Nothing
    q = Nothing
    new(frames, joints, tfs, q)
  end
end



fr_world = Frame("world")
fr_body1 = Frame("body1")
fr_body2 = Frame("body2")

joint1 = Revolute("world", "body1", [0, 0, 0], "joint1")
joint2 = Revolute("body1", "body2", [1, 0, 0], "joint2")

frame_list = [fr_world, fr_body1, fr_body2]
joint_list = [joint1, joint2]

r = Robot([fr_world, fr_body1, fr_body2],
          [joint1, joint2]) 

function Rot2d(theta)
  m = [cos(theta) sin(theta);
       -sin(theta) cos(theta)]
end

function set_configuration(r::Robot, q)
  # precompute minimal transformation 
  tfs = Dict()
  joints = collect(values(r.joints))
  for jt in joints, angle in q
    key = (jt.child, jt.parent)
    tf(vec) = Rot2d(angle)*vec[1:2] + jt.origin[1:2]
    tfs[key] = tf
  end
  r.tfs = tfs
end

function get_parent_frame(r::Robot, frame::Frame)
  joint_parent = r.joints[frame.parent.name]
  println(joint_parent.parent)
  frame_parent = r.frames[joint_parent.parent]
end

function get_tf(r::Robot, frame_desc_name, frame_asc_name)
  key_try = (frame_desc_name, frame_asc_name)
  try
    tf = r.tfs[key_try]
    return tf
  catch error
    if isa(error, KeyError) # key not found
      frame_asc = r.frames[frame_asc_name]
      frame_desc = r.frames[frame_desc_name]

      function recursion(vec, frame)
        frame == frame_asc && (return vec)

        frame_parent = get_parent_frame(r, frame)
        name_this = frame.name
        name_parent = frame_parent.name

        key = (name_this, name_parent)
        tf_single = r.tfs[key] #lambda
        recursion(tf_single(vec), frame_parent)
      end
      tf(vec) = recursion(vec, frame_desc)
      r.tfs[(frame_desc_name, frame_asc_name)] = tf
      return tf
    end
  end
end

set_configuration(r, [0, 0])
a = r.tfs[("body1", "world")]([0, 1])
b = r.tfs[("body2", "body1")]([1, 0])

tf = get_tf(r, "body1", "world")
tf([1, 0])


  
  
  
