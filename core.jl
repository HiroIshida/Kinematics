include("./transform.jl")

abstract type AbstractJoint end

mutable struct Link
  parent::Union{Nothing, AbstractJoint}
  child::Union{Nothing, AbstractJoint}
  name
  length
  color
  function Link(name; length=1.0, color=:red)
    new(Nothing, Nothing, name, length, color)
  end
end

mutable struct Revolute <: AbstractJoint
  parent::Union{Nothing, Link}
  child::Union{Nothing, Link}
  origin::Transform
  name
  function Revolute(parent, child, origin, name)
    new(parent, child, origin, name)
  end
end


