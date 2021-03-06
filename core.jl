include("./transform.jl")

import LightXML
import LightXML: parse_file, child_nodes, child_elements, attribute

abstract type AbstractJoint end

mutable struct Link
    parent::Union{Nothing, AbstractJoint}
    child::Union{Nothing, AbstractJoint}
    name
    length
    color
    function Link(name; length=1.0, color=:red)
        new(nothing, nothing, name, length, color)
    end
end

function get_parent_link(link::Link)
    parent = link.parent # joint
    isnothing(parent) && return nothing
    # note that joint always has parent
    return parent.parent
end

function get_child_link(link::Link)
    child = link.child # joint
    isnothing(child) && return nothing
    # note that joint always has child
    return child.child
end

mutable struct Revolute <: AbstractJoint
    name
    parent::Union{Nothing, Link}
    child::Union{Nothing, Link}
    origin::Transform # T: joint -> parent_link
    axis::SVector{3, Float64}
    function Revolute(name, parent, child, origin, axis)
        new(name, parent, child, origin, axis)
    end
end

function get_tf_adj_links(jt::Revolute, angle)
    link_parent = jt.parent 
    link_child = jt.child
    tf_joint2plink = jt.origin
    tf_clink2joint = Transform([0, 0, 0], jt.axis, angle)
    tf_clink2plink = tf_clink2joint ∘ tf_joint2plink
    return tf_clink2plink
end

function parse_urdf(urdffile)
    xroot = LightXML.root(parse_file(urdffile))

    link_xml_list = []
    joint_xml_list = []
    for elem in child_elements(xroot)
        elem_name = LightXML.name(elem)
        if elem_name == "link" 
            push!(link_xml_list, elem)
        elseif elem_name == "joint"
            push!(joint_xml_list, elem)
        end
    end

    # because link and joint are mutual referential, 
    # (step1) we first initialize links without fields of joint (parent, child)
    # (step2) then, after creating all joint instances, 
    # (step3) we then connect link and joint
    
    # step1
    link_dict = Dict()
    for link_xml in link_xml_list
        link_name = attribute(link_xml, "name")
        link = Link(link_name)
        link_dict[link_name] = link
    end

    joint_dict = Dict()
    for joint_xml in joint_xml_list
        # step2
        joint_name, parent_name, child_name, origin, axis = parse_joint_xml(joint_xml)
        joint = Revolute(joint_name, link_dict[parent_name], link_dict[child_name],
                         origin, axis)
        joint_dict[joint_name] = joint

        # step3 
        link_dict[parent_name].child = joint
        link_dict[child_name].parent = joint
    end

    return link_dict, joint_dict
end

function parse_joint_xml(joint_xml)
    joint_name = attribute(joint_xml, "name")
    parent_name = child_name = origin = axis = nothing

    parse_arrstr(str) = parse.(Float64, split(str))

    for elem in child_elements(joint_xml)
        elem_name = LightXML.name(elem)
        if elem_name == "parent"
            parent_name = attribute(elem, "link")
        elseif elem_name == "child"
            child_name = attribute(elem, "link")
        elseif elem_name == "origin"
            xyz = parse_arrstr(attribute(elem, "xyz"))
            rpy = parse_arrstr(attribute(elem, "rpy"))
            origin = Transform(xyz, rpy)
        elseif elem_name == "axis"
            axis = parse_arrstr(attribute(elem, "xyz"))
        end
    end
    return joint_name, parent_name, child_name, origin, axis
end

mutable struct Robot
    link_dict::Dict{String, Link}
    joint_dict::Dict{String, AbstractJoint}
    tf_dict::Dict{Tuple{String, String}, Transform}

    n_link
    n_joint
end

function Robot(urdffile)
    link_dict, joint_dict = parse_urdf(urdffile)
    n_link = length(link_dict)
    n_joint = length(joint_dict) # TODO subtract number of fixed joints
    tf_dict = Dict()
    Robot(link_dict, joint_dict, tf_dict, n_link, n_joint) 
end

function set_configuration(r::Robot, q)
    @assert length(q) == r.n_joint 

    tf_dict = Dict()
    for (key_joint, angle) in zip(keys(r.joint_dict), q)
        joint = r.joint_dict[key_joint]
        tf_plink2clink = get_tf_adj_links(joint, angle)

        tf_key = (joint.child.name, joint.parent.name) # (from, to)
        tf_dict[tf_key] = tf_plink2clink
    end
    r.tf_dict = tf_dict
end

function get_tf(r::Robot, from::Link, to::Link)
    # * recursion
    key_try = (from.name, to.name)
    if ~haskey(r.tf_dict, key_try)
        # from -> middle -> to
        middle = get_parent_link(from)
        tf_upper2to = get_tf(r, middle, to)
        tf_from2upper = get_tf(r, from, middle) # this must be exist because they are adj
        tf_from2to = tf_from2upper ∘ tf_upper2to
        r.tf_dict[key_try] = tf_from2to
    end
    return r.tf_dict[key_try]
end

function get_tf(r::Robot, from::String, to::String)
    get_tf(r, r.link_dict[from], r.link_dict[to])
end

r = Robot("sample.urdf")
@time begin 
    for i in 1:10000
        set_configuration(r, [0.5, 0.3])
        get_tf(r, "link2", "world")
    end
end
