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

function tf_adj_links(jt::Revolute, angle)
    link_parent = jt.parnet 
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
    link_dict
    joint_dict 
    tf_dict

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
    for (key_joint, angle) in zip(key(r.joint_dict), q)
        r.joint_dict[key_joint].angle = q
    end
end

r = Robot("sample.urdf")
