using Pkg
open("REQUIRE_FOR_ROBOT") do file
    for l in eachline(file)
        Pkg.add(l)
    end
end

Pkg.add(PackageSpec(url="https://github.com/Nate711/PiGPIO.jl"))
