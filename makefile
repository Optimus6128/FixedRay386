obj = demo.obj timer.obj tinyfont.obj video.obj mathutil.obj gfxtools.obj fx_ray.obj
bin = demo.exe

CPP = wpp386
CXXFLAGS = -3 -fp3 -otexan -oh -oi -ei -bt=DOS
LD = wlink

$(bin): $(obj)
    $(LD) name $@ file { *.obj } $(LDFLAGS)

.cpp.obj: .autodepend
    $(CPP) $(CXXFLAGS) $<

clean: .symbolic
    del *.obj
    del *.err
    del $(bin)
