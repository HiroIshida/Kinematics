using PackageCompiler
import Plots

create_sysimage(:Plots; sysimage_path="plots.so")
