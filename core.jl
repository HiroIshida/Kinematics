include("./transform.jl")

#abstract struct AbstractJoint end

mutable struct Revolute{T} #<: AbstractJoint
  """
  as for mutually recursive struct in Julia see:
  https://discourse.julialang.org/t/mutually-recursive-type/25536/2?u=hiroishida
  this may be got better in some future..
  """ 
  parent::Union{Nothing, T}
  child::Union{Nothing, T}
  origin::Transform
  name
  function Revolute(parent, child, origin, name)
    new{Link}(parent, child, origin, name)
  end
end

mutable struct Link
  parent::Union{Nothing, Revolute}
  child::Union{Nothing, Revolute}
  name
  length
  color
  function Link(name; length=1.0, color=:red)
    new(Nothing, Nothing, name, length, color)
  end
end

