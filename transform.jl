using LinearAlgebra
using StaticArrays



#=
about overloading getproperty
https://discourse.julialang.org/t/whats-the-difference-between-fields-and-properties/12495
=#

Rx(a) = @SMatrix [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]
Ry(a) = @SMatrix [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]
Rz(a) = @SMatrix [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]

function quaternion2matrix(q)  
  q1, q2, q3, q0 = q
  mat = @SMatrix[q0^2+q1^2-q2^2-q3^2 2*(q1*q2-q0*q3) 2*(q1*q3+q0*q2);
                 2*(q1*q2+q0*q3) q0^2-q1^2+q2^2-q3^2 2*(q2*q3-q0*q1);
                 2*(q1*q3-q0*q2) 2*(q2*q3+q0*q1) q0^2-q1^2-q2^2+q3^2]
  return mat
end

#=
function rpy2quaternion(r, p, y)
  rpy = @SVector [r, p, y]
  mat = Rz(-r) * Ry(-p) * Rx(-y)
  Euler(rpy, mat)
end
=#

mutable struct Quaternion  
  vec::SVector{4, Float64}
  mat::Union{Nothing, SMatrix{3, 3, Float64}}
  function Quaternion(x, y, z, w)
    vec = @SVector [x, y, z, w]
    new(vec, nothing)
  end
end

function Base.getproperty(q::Quaternion, field::Symbol)
  if field == :mat   
    if isnothing(getfield(q, :mat))
      vec = getfield(q, :vec)
      q.mat = quaternion2matrix(vec)
    end
  end
  return getfield(q, field)
end

function (q::Quaternion)(x) 
  q.mat * x 
end

struct Transform
  trans::SVector{3, Float64}
  rot::Quaternion
end

function Transform(vec)
  if length(vec) == 7
    trans = @SVector[vec[1], vec[2], vec[3]]
    rot = Quaternion(vec[4], vec[5], vec[6], vec[7])
    Transform(trans, rot)
  else
    error("vec must be dim=6 or 7")
  end
end

function (tf::Transform)(x)
  return tf.trans + tf.rot(x)
end


#tf1 = Transform([0, 0, 0, 0, 0, 0])
tf2 = Transform([0, 0, 0, 0, 0, 0, 1])
tf2([1, 1, 1])

