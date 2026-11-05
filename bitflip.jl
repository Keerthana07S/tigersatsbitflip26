# this is the code for faster propogation of our Markov Chain. 
# I decided to use Julia b/c as we propogate through multiple chips this can start to become more computationally intensive with Python or C++
# Also, Julia already has a lot of cool libraries for this stuff. 
# Keerthana Srinivasan
# October 23, 2026
const LIB = joinpath(@__DIR__, "libbitflip.so")
using LinearAlgebra

h = ccall((:bf_open, LIB), Ptr{Cvoid}, (Cstring, Cint), "/dev/i2c-1", 1)   
h == C_NULL && error("bf_open failed")
ccall((:bf_write_patterns, LIB), Cint, (Ptr{Cvoid},), h)

for _ in 1:50
    ccall((:bf_sim_inject, LIB), Cvoid, (Ptr{Cvoid}, Cint), h, 3)
    ccall((:bf_scan, LIB), Cint, (Ptr{Cvoid},), h)
end

# we call our matrix from the .hpp code here.
markov(dev) = begin
    buf = zeros(Float64, 16)
    ccall((:bf_markov, LIB), Cvoid, (Ptr{Cvoid}, Cint, Ptr{Float64}), h, dev, buf)
    permutedims(reshape(buf, 4, 4))   
end

ndev = ccall((:bf_num_devices, LIB), Cint, (Ptr{Cvoid},), h)
for d in 0:ndev-1
    name = unsafe_string(ccall((:bf_device_name, LIB), Cstring, (Ptr{Cvoid}, Cint), h, d))
    P = markov(d)
    println(name, "  P(clean→upset within 10 scans) = ", round(1 - (P^10)[1, 1], digits = 3))
end
ccall((:bf_close, LIB), Cvoid, (Ptr{Cvoid},), h)
