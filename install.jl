using Pkg
open("REQUIRE") do file
    for l in eachline(file)
        Pkg.add(l)
    end
end

Pkg.add(PackageSpec(url="https://github.com/Nate711/PiGPIO.jl"))
Pkg.add(PackageSpec(url="https://github.com/klowrey/MuJoCo.jl"))