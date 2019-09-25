__precompile__()

module WooferSim

import GLFW
using MuJoCo
using StaticArrays
using PyCall
using Rotations
using Parameters
using LinearAlgebra
using Plots

include("Controller.jl")
include("Types.jl")

##################################################### globals
const fontscale = mj.FONTSCALE_200 # can be 100, 150, 200
const maxgeom       = 5000 # preallocated geom array in mjvScene
const syncmisalign  = 0.1  # maximum time mis-alignment before re-sync
const refreshfactor = 0.7  # fraction of refresh available for simulation

# modified from https://github.com/klowrey/MujocoSim.jl/
mutable struct mjSim
   # visual interaction controls
   lastx::Float64
   lasty::Float64
   button_left::Bool
   button_middle::Bool
   button_right::Bool

   lastbutton::GLFW.MouseButton
   lastclicktm::Float64

   refreshrate::Int

   showhelp::Int
   showoption::Bool
   showdepth::Bool
   showfullscreen::Bool
   showsensor::Bool
   slowmotion::Bool

   showinfo::Bool
   paused::Bool
   keyreset::Int
   record::Any
   vidbuff::Vector{UInt8}

   framecount::Float64
   framenum::Int
   lastframenum::Int

   # MuJoCo things
   scn::Ref{mjvScene}
   cam::Ref{mjvCamera}
   vopt::Ref{mjvOption}
   pert::Ref{mjvPerturb}
   con::Ref{mjrContext}
   figsensor::Ref{mjvFigure}
   m::jlModel
   d::jlData

   # GLFW handle
   window::GLFW.Window
   vmode::GLFW.VidMode

   #uistate::mjuiState
   #ui0::mjUI
   #ui1::mjUI

   function mjSim(m::jlModel, d::jlData, name::String;
                  width=0, height=0)
      vmode = GLFW.GetVideoMode(GLFW.GetPrimaryMonitor())
      w = width > 0 ? width : floor(2*vmode.width / 3)
      h = height > 0 ? height : floor(2*vmode.height / 3)
      new(0.0, 0.0, false, false, false, GLFW.MOUSE_BUTTON_1, 0.0,
          vmode.refreshrate,
          0, false, false, false, false, false, true, true, 0,
          nothing, #open("/tmp/test.bin","w"),
          Vector{UInt8}(undef, 5120 * 2880 * 3),
      0.0, 0, 0,
      Ref(mjvScene()),
      Ref(mjvCamera()),
      Ref(mjvOption()),
      Ref(mjvPerturb()),
      Ref(mjrContext()),
      Ref(mjvFigure()),
      m, d,
      GLFW.CreateWindow(w, h, "Simulate"),
      vmode
     )
   end
end

export mjSim

const keycmds = Dict{GLFW.Key, Function}(GLFW.KEY_F1=>(s)->begin  # help
                                            s.showhelp += 1
                                            if s.showhelp > 2 s.showhelp = 0 end
                                         end,
                                         GLFW.KEY_F2=>(s)->begin  # option
                                            s.showoption = !s.showoption;
                                         end,
                                         GLFW.KEY_F3=>(s)->begin  # info
                                            s.showinfo = !s.showinfo;
                                         end,
                                         GLFW.KEY_F4=>(s)->begin  # depth
                                            s.showdepth = !s.showdepth;
                                         end,
                                         GLFW.KEY_F5=>(s)->begin  # toggle full screen
                                            s.showfullscreen = !s.showfullscreen;
                                            s.showfullscreen ? GLFW.MaximizeWindow(s.window) : GLFW.RestoreWindow(s.window)
                                         end,
                                         #GLFW.KEY_F6=>(s)->begin  # stereo
                                         #   s.stereo = s.scn.stereo == mj.mjSTEREO_NONE ? mjSTEREO_QUADBUFFERED : mj.mjSTEREO_NONE
                                         #   s.scn[].stereo
                                         #end,
                                         GLFW.KEY_F7=>(s)->begin  # sensor figure
                                            s.showsensor = !s.showsensor;
                                         end,
                                         GLFW.KEY_F8=>(s)->begin  # profiler
                                            s.showprofiler = !s.showprofiler;
                                         end,
                                         GLFW.KEY_ENTER=>(s)->begin  # slow motion
                                            s.slowmotion = !s.slowmotion;
                                            s.slowmotion ? println("Slow Motion Mode!") : println("Normal Speed Mode!")
                                         end,
                                         GLFW.KEY_SPACE=>(s)->begin  # pause
                                            s.paused = !s.paused
                                            s.paused ? println("Paused") : println("Running")
                                         end,
                                         GLFW.KEY_V=>(s)->begin
                                            if s.record == nothing
                                               println("Saving video to /tmp/video.mp4")
                                               #wi, hi = GLFW.GetFramebufferSize(w)
                                               s.record = open(`ffmpeg -y
                                                               -f rawvideo -pixel_format rgb24
                                                               -video_size 1280x800 -framerate 60
                                                               -i pipe:0
                                                               -preset fast -threads 0
                                                               -vf "vflip" /tmp/video.mp4`, "w")
                                            else
                                               println("Closing /tmp/video.mp4")
                                               close(s.record)
                                               s.record = nothing
                                            end
                                         end,
                                         GLFW.KEY_PAGE_UP=>(s)->begin    # previous keyreset
                                            s.keyreset = min(s.m.m[].nkey - 1, s.keyreset + 1)
                                         end,
                                         GLFW.KEY_PAGE_DOWN=>(s)->begin  # next keyreset
                                            s.keyreset = max(-1, s.keyreset - 1)
                                         end,
                                         # continue with reset
                                         GLFW.KEY_BACKSPACE=>(s)->begin  # reset
                                            GLFW.DestroyWindow(s.window)
                                            return nothing
                                          #   mj_resetData(s.m.m, s.d.d)
                                          #   if s.keyreset >= 0 && s.keyreset < s.m.m[].nkey
                                          #      s.d[].time = s.m.key_time[s.keyreset+1]
                                          #      s.d.qpos[:] = s.m.key_qpos[:,s.keyreset+1]
                                          #      s.d.qvel[:] = s.m.key_qvel[:,s.keyreset+1]
                                          #      s.d.act[:]  = s.m.key_act[:,s.keyreset+1]
                                          #   end
                                          #   mj_forward(s.m, s.d)
                                          #   #profilerupdate()
                                          #   sensorupdate(s)
                                         end,
                                         GLFW.KEY_RIGHT=>(s)->begin  # step forward
                                            if s.paused
                                               mj_step(s.m, s.d)
                                               #profilerupdate()
                                               sensorupdate(s)
                                            end
                                         end,
                                         # GLFW.KEY_LEFT=>(s)->begin  # step back
                                         #    if s.paused
                                         #       dt = s.m.m[].opt.timestep
                                         #       s.m.m[].opt.timestep = -dt
                                         #       #cleartimers(s.d);
                                         #       mj_step(s.m, s.d);
                                         #       s.m.m[].opt.timestep = dt
                                         #       #profilerupdate()
                                         #       sensorupdate(s)
                                         #    end
                                         # end,
                                         GLFW.KEY_DOWN=>(s)->begin  # step forward 100
                                            if s.paused
                                               #cleartimers(d);
                                               for n=1:100 mj_step(s.m, s.d) end
                                               #profilerupdate();
                                               sensorupdate(s)
                                            end
                                         end,
                                         # GLFW.KEY_UP=>(s)->begin  # step back 100
                                         #    if s.paused
                                         #       dt = s.m.m[].opt.timestep
                                         #       s.m.m[].opt.timestep = -dt
                                         #       #cleartimers(d)
                                         #       for n=1:100 mj_step(s.m, s.d) end
                                         #       s.m.m[].opt.timestep = dt
                                         #       #profilerupdate();
                                         #       sensorupdate(s)
                                         #    end
                                         # end,
                                         GLFW.KEY_ESCAPE=>(s)->begin  # free camera
                                            s.cam[]._type = Int(mj.CAMERA_FREE)
                                         end,
                                         GLFW.KEY_EQUAL=>(s)->begin  # bigger font
                                            if fontscale < 200
                                               fontscale += 50
                                               mjr_makeContext(s.m.m, s.con, fontscale)
                                            end
                                         end,
                                         GLFW.KEY_MINUS=>(s)->begin  # smaller font
                                            if fontscale > 100
                                               fontscale -= 50;
                                               mjr_makeContext(s.m.m, s.con, fontscale);
                                            end
                                         end,
                                         GLFW.KEY_LEFT_BRACKET=>(s)->begin  # '[' previous fixed camera or free
                                            fixedcam = s.cam._type
                                            if s.m.m[].ncam > 0 && fixedcam == Int(mj.CAMERA_FIXED)
                                               fixedcamid = s.cam.fixedcamid
                                               if (fixedcamid  > 0)
                                                  s.cam[].fixedcamid = fixedcamid-1
                                               else
                                                  s.cam[]._type = Int(mj.CAMERA_FREE)
                                               end
                                            end
                                         end,
                                         GLFW.KEY_RIGHT_BRACKET=>(s)->begin  # ']' next fixed camera
                                            if s.m.m[].ncam > 0
                                               fixedcam = s.cam.type
                                                  fixedcamid = s.cam.fixedcamid
                                                  if fixedcam != Int(mj.CAMERA_FIXED)
                                                     s.cam[]._type = Int(mj.CAMERA_FIXED)
                                                  elseif fixedcamid < s.m.m[].ncam - 1
                                                     s.cam[].fixedcamid = fixedcamid+1
                                                  end
                                               end
                                            end,
                                            GLFW.KEY_SEMICOLON=>(s)->begin  # cycle over frame rendering modes
                                               frame = s.vopt.frame
                                               s.vopt[].frame = max(0, frame - 1)
                                            end,
                                            GLFW.KEY_APOSTROPHE=>(s)->begin  # cycle over frame rendering modes
                                               frame = s.vopt.frame
                                               s.vopt[].frame = min(Int(mj.NFRAME)-1, frame+1)
                                            end,
                                            GLFW.KEY_PERIOD=>(s)->begin  # cycle over label rendering modes
                                               label = s.vopt.label
                                               s.vopt[].label = max(0, label-1)
                                            end,
                                            GLFW.KEY_SLASH=>(s)->begin  # cycle over label rendering modes
                                               label = s.vopt.label
                                               s.vopt[].label = min(Int(mj.NLABEL)-1, label+1)
                                            end)

##################################################### functions
function alignscale(s::mjSim)
   s.cam[].lookat = s.m.m[].stat.center
   s.cam[].distance = 1.5*s.m.m[].stat.extent

   # set to free camera
   s.cam[]._type = Cint(mj.CAMERA_FREE)
end

function str2vec(s::String, len::Int)
   str = zeros(UInt8, len)
   str[1:length(s)] = codeunits(s)
   return str
end

# init sensor figure
function sensorinit(s::mjSim)
   # set figure to default
   mjv_defaultFigure(s.figsensor)

   # set flags
   s.figsensor[].flg_extend = Cint(1)
   s.figsensor[].flg_barplot = Cint(1)

   s.figsensor[].title = str2vec("Sensor data", length(s.figsensor[].title))

   # y-tick nubmer format
   s.figsensor[].yformat = str2vec("%.0f", length(s.figsensor[].yformat))

   # grid size
   s.figsensor[].gridsize = [2, 3]

   # minimum range
   s.figsensor[].range = [[0 1],[-1 1]]
end

# update sensor figure
function sensorupdate(s::mjSim)
   maxline = 10

   for i=1:maxline # clear linepnt
      mj.set(s.figsensor, :linepnt, Cint(0), i)
   end

   lineid = 1 # start with line 0
   m = s.m
   d = s.d

   # loop over sensors
   for n=1:m.m[].nsensor
      # go to next line if type is different
      if (n > 1 && m.sensor_type[n] != m.sensor_type[n - 1])
         lineid = min(lineid+1, maxline)
      end

      # get info about this sensor
      cutoff = m.sensor_cutoff[n] > 0 ? m.sensor_cutoff[n] : 1.0
      adr = m.sensor_adr[n]
      dim = m.sensor_dim[n]

      # data pointer in line
      p = mj.get(s.figsensor, :linepnt, lineid)

      # fill in data for this sensor
      for i=0:(dim-1)
         # check size
         if ((p + 2i) >= Int(mj.MAXLINEPNT) / 2) break end

         x1 = 2p + 4i + 1
         x2 = 2p + 4i + 3
         mj.set(s.figsensor, :linedata, adr+i, lineid, x1)
         mj.set(s.figsensor, :linedata, adr+i, lineid, x2)

         y1 = 2p + 4i + 2
         y2 = 2p + 4i + 4
         se = d.sensordata[adr+i+1]/cutoff
         mj.set(s.figsensor, :linedata,  0, lineid, y1)
         mj.set(s.figsensor, :linedata, se, lineid, y2)
      end

      # update linepnt
      mj.set(s.figsensor, :linepnt,
             min(Int(mj.MAXLINEPNT)-1, p+2dim),
             lineid)
   end
end

# show sensor figure
function sensorshow(s::mjSim, rect::mjrRect)
   # render figure on the right
   viewport = mjrRect(rect.width - rect.width / 4,
                      rect.bottom,
                      rect.width / 4,
                      rect.height / 3)
   mjr_figure(viewport, s.figsensor, s.con)
end

##################################################### callbacks

function mykeyboard(s::mjSim, window::GLFW.Window,
                    key::GLFW.Key, scancode::Int32, act::GLFW.Action, mods::Int32)
   # do not act on release
   if act == GLFW.RELEASE return end

   try
      keycmds[key](s) # call anon function in Dict with s struct passed in
   catch
      # control keys
      if mods & GLFW.MOD_CONTROL > 0
         if key == GLFW.KEY_A
            alignscale(s)
         elseif key == GLFW.KEY_P
            println(s.d.qpos)
            #elseif key == GLFW.KEY_L && lastfile[0]
            #   loadmodel(window, s.)
         elseif key == GLFW.KEY_Q
            if s.record != nothing
               close(s.record)
               s.record = nothing
            end
            GLFW.SetWindowShouldClose(window, true)
         end
      end

      # toggle visualiztion flag
      for i=1:Int(mj.NVISFLAG)
         if Int(key) == Int(mj.VISSTRING[i,3][1])
            flags = MVector(s.vopt[].flags)
            flags[i] = flags[i] == 0 ? 1 : 0
            s.vopt[].flags = flags
            return
         end
      end
      # toggle rendering flag
      for i=1:Int(mj.NRNDFLAG)
         if Int(key) == Int(mj.RNDSTRING[i,3][1])
            flags = MVector(s.scn[].flags)
            flags[i] = flags[i] == 0 ? 1 : 0
            s.scn[].flags = flags
            return
         end
      end
      # toggle geom/site group
      for i=1:Int(mj.NGROUP)
         if Int(key) == i + Int('0')
            if mods & GLFW.MOD_SHIFT == true
               sitegroup = MVector(s.vopt[].sitegroup)
               sitegroup[i] = sitegroup[i] > 0 ? 0 : 1
               s.vopt[].sitegroup[i] = sitegroup
               return
            else
               geomgroup = MVector(s.vopt[].geomgroup)
               geomgroup[i] = geomgroup[i] > 0 ? 0 : 1
               s.vopt[].geomgroup = geomgroup
               return
            end
         end
      end
   end
end

function mouse_move(s::mjSim, window::GLFW.Window,
                    xpos::Float64, ypos::Float64)
   # no buttons down: nothing to do
   if !s.button_left && !s.button_middle && !s.button_right
      return
   end

   # compute mouse displacement, save
   dx = xpos - s.lastx
   dy = ypos - s.lasty
   s.lastx = xpos
   s.lasty = ypos

   width, height = GLFW.GetWindowSize(window)

   mod_shift = GLFW.GetKey(window, GLFW.KEY_LEFT_SHIFT) || GLFW.GetKey(window, GLFW.KEY_RIGHT_SHIFT)

   # determine action based on mouse button
   if s.button_right
      action = mod_shift ? Int(mj.MOUSE_MOVE_H) : Int(mj.MOUSE_MOVE_V)
   elseif s.button_left
      action = mod_shift ? Int(mj.MOUSE_ROTATE_H) : Int(mj.MOUSE_ROTATE_V)
   else
      action = Int(mj.MOUSE_ZOOM)
   end

   # move perturb or camera
   if s.pert[].active != 0
      mjv_movePerturb(s.m.m, s.d.d, action,
                      dx / height, dy / height,
                      s.scn, s.pert);
   else
      mjv_moveCamera(s.m.m, action,
                     dx / height, dy / height,
                     s.scn, s.cam)
   end
end

# past data for double-click detection
function mouse_button(s::mjSim, window::GLFW.Window,
                      button::GLFW.MouseButton, act::GLFW.Action, mods::Int32)
   # update button state
   s.button_left = GLFW.GetMouseButton(window, GLFW.MOUSE_BUTTON_LEFT)
   s.button_middle = GLFW.GetMouseButton(window, GLFW.MOUSE_BUTTON_MIDDLE)
   s.button_right = GLFW.GetMouseButton(window, GLFW.MOUSE_BUTTON_RIGHT)

   # Alt: swap left and right
   if mods == GLFW.MOD_ALT
      tmp = s.button_left
      s.button_left = s.button_right
      s.button_right = tmp

      if button == GLFW.MOUSE_BUTTON_LEFT
         button = GLFW.MOUSE_BUTTON_RIGHT;
      elseif button == GLFW.MOUSE_BUTTON_RIGHT
         button = GLFW.MOUSE_BUTTON_LEFT;
      end
   end

   # update mouse position
   x, y = GLFW.GetCursorPos(window)
   s.lastx = x
   s.lasty = y

   # set perturbation
   newperturb = 0;
   if act == GLFW.PRESS && mods == GLFW.MOD_CONTROL && s.pert[].select > 0
      # right: translate;  left: rotate
      if s.button_right
         newperturb = Int(mj.PERT_TRANSLATE)
      elseif s.button_left
         newperturb = Int(mj.PERT_ROTATE)
      end
      # perturbation onset: reset reference
      if newperturb>0 && s.pert[].active==0
         mjv_initPerturb(s.m.m, s.d.d, s.scn, s.pert)
      end
   end
   s.pert[].active = newperturb

   # detect double-click (250 msec)
   if act == GLFW.PRESS && (time() - s.lastclicktm < 0.25) && (button == s.lastbutton)
      # determine selection mode
      if button == GLFW.MOUSE_BUTTON_LEFT
         selmode = 1;
      elseif mods == GLFW.MOD_CONTROL
         selmode = 3; # CTRL + Right Click
      else
         selmode = 2; # Right Click
      end
      # get current window size
      width, height = GLFW.GetWindowSize(window)

      # find geom and 3D click point, get corresponding body
      selpnt = zeros(3)
      selgeom, selskin = Int32(0), Int32(0)
      selbody = mjv_select(s.m.m, s.d.d, s.vopt,
                           width / height, x / width,
                           (height - y) / height,
                           s.scn, selpnt, selgeom, selskin)

      # set lookat point, start tracking is requested
      if selmode == 2 || selmode == 3
         # copy selpnt if geom clicked
         if selbody >= 0
            s.cam[].lookat = SVector{3,Float64}(selpnt...)
         end

         # switch to tracking camera
         if selmode == 3 && selbody >= 0
            s.cam[]._type = Int(mj.CAMERA_TRACKING)
            s.cam[].trackbodyid = selbody
            s.cam[].fixedcamid = -1
         end
      else # set body selection
         if selbody >= 0
            # compute localpos
            tmp = selpnt - s.d.xpos[:,selbody+1]
            res = reshape(s.d.xmat[:,selbody+1], 3, 3)' * tmp
            s.pert[].localpos = SVector{3}(res)

            # record selection
            s.pert[].select = selbody
            s.pert[].skinselect = selskin
         else
            s.pert[].select = 0
            s.pert[].skinselect = -1
         end
      end

      # stop perturbation on select
      s.pert[].active = 0
   end
   # save info
   if act == GLFW.PRESS
      s.lastbutton = button
      s.lastclicktm = time()
   end
end

function scroll(s::mjSim, window::GLFW.Window,
                xoffset::Float64, yoffset::Float64)
   # scroll: emulate vertical mouse motion = 5% of window height
   mjv_moveCamera(s.m.m, Int(mj.MOUSE_ZOOM),
                  0.0, -0.05 * yoffset, s.scn, s.cam);

end

function drop(window::GLFW.Window,
              count::Int, paths::String)
end

function start(mm::jlModel, dd::jlData, width=1200, height=900) # TODO named args for callbacks
   GLFW.WindowHint(GLFW.SAMPLES, 4)
   GLFW.WindowHint(GLFW.VISIBLE, 1)

   s = mjSim(mm, dd, "Simulate"; width=width, height=height)

   @info("Refresh Rate: $(s.refreshrate)")
   @info("Resolution: $(width)x$(height)")

   # Make the window's context current
   GLFW.MakeContextCurrent(s.window)
   GLFW.SwapInterval(1)

   # init abstract visualization
   mjv_defaultCamera(s.cam)
   mjv_defaultOption(s.vopt)
   #profilerinit();
   sensorinit(s)

   # make empty scene
   mjv_defaultScene(s.scn)
   mjv_makeScene(s.m, s.scn, maxgeom)

   # mujoco setup
   mjv_defaultPerturb(s.pert)
   mjr_defaultContext(s.con)
   mjr_makeContext(s.m, s.con, Int(fontscale)) # model specific setup

   alignscale(s)
   mjv_updateScene(s.m, s.d,
                   s.vopt, s.pert, s.cam, Int(mj.CAT_ALL), s.scn)

   GLFW.SetKeyCallback(s.window, (w,k,sc,a,m)->mykeyboard(s,w,k,sc,a,m))

   GLFW.SetCursorPosCallback(s.window, (w,x,y)->mouse_move(s,w,x,y))
   GLFW.SetMouseButtonCallback(s.window, (w,b,a,m)->mouse_button(s,w,b,a,m))
   GLFW.SetScrollCallback(s.window, (w,x,y)->scroll(s,w,x,y))
   ##GLFW.SetDropCallback(s.window, drop)

   return s
end


#### To customize what is rendered, change the following functions

function render(s::WooferSim.mjSim, w::GLFW.Window)
   wi, hi = GLFW.GetFramebufferSize(w)
   rect = mjrRect(Cint(0), Cint(0), Cint(wi), Cint(hi))
   smallrect = mjrRect(Cint(0), Cint(0), Cint(wi), Cint(hi))

   # update scene
   mjv_updateScene(s.m, s.d,
                   s.vopt, s.pert, s.cam, Int(mj.CAT_ALL), s.scn)
   # render
   mjr_render(rect, s.scn, s.con)

   if s.record != nothing
      mjr_readPixels(s.vidbuff, C_NULL, rect, s.con);
      write(s.record, s.vidbuff[1:3*rect.width*rect.height]);
   end

   # Swap front and back buffers
   GLFW.SwapBuffers(w)
end

function loadmodel(modelfile, width, height)
   ptr_m = mj_loadXML(modelfile, C_NULL)
   ptr_d = mj_makeData(ptr_m)
   m, d = mj.mapmujoco(ptr_m, ptr_d)
   mj_forward(m, d)
   s = WooferSim.start(m, d, width, height)
   @info("Model file: $modelfile")

   GLFW.SetWindowRefreshCallback(s.window, (w)->render(s,w))

   return s
end

function simulate()
   # Look at local directory
   pushfirst!(PyVector(pyimport("sys")."path"), @__DIR__)

   # Parse MuJoCo XML file
   xmlparser = pyimport("PupperXMLParser")
   xmlparser.Parse()

   # Load robot model
   s = loadmodel("src/pupper_out.xml", 1200, 900)

   d = s.d
   m = s.m

   ################################ CONTROLLER CONFIGURATION ##############################
   #= NOTES
   vxyref: Reference velocity in the x and y directions
   zref: Reference z coordinate of the feet during stance relative to the body
   wzref: Reference angular velocity in the z axis (aka yaw)

   zclearance: How far the robot should pick up its feet during swing. Relative to the ground.

   Δx: Default x coordinate (front-back) of the feet if the robot were to stand still.  
   Δy: Default y coordinate (side to side) of the feet if the robot were to stand still
   =#

   controller::Controller = Controller()

   # Uncomment for Crab settings
   # controller.mvref = MovementReference(vxyref=SVector(0.05,0.2), zref=-0.08, wzref=0.0)
   # controller.swingparams = SwingParams(zclearance=0.02)
   # controller.stanceparams = StanceParams(Δx=0.13, Δy=0.16)

   # Uncomment for Puppy settings
   # controller.mvref = MovementReference(vxyref=SVector(0.0,0.15), zref=-0.15, wzref=-0.8)
   # controller.swingparams = SwingParams(zclearance=0.02)
   # controller.stanceparams = StanceParams(Δx=0.1, Δy=0.09)

   controller.mvref = MovementReference(vxyref=SVector(0.2,0.0), zref=-0.15, wzref=0.0)
   controller.swingparams = SwingParams(zclearance=0.02)
   controller.stanceparams = StanceParams(Δx=0.1, Δy=0.09)

   ################################ END CONTROLLER CONFIGURATION ###########################

   simulationsteps_per_controlstep::Int = round(controller.gaitparams.dt / m.m[].opt.timestep)
   simulationstep::Int = 0

   # Loop until the user closes the window
   WooferSim.alignscale(s)
   while !GLFW.WindowShouldClose(s.window)
      ### basically sim step so things don't have to be defined multiple times
      if s.paused
         if s.pert[].active > 0
            mjv_applyPerturbPose(m, d, s.pert, 1)  # move mocap and dynamic bodies
            mj_forward(m, d)
         end
      else
         #slow motion factor: 10x
         factor = s.slowmotion ? 10 : 1

         # advance effective simulation time by 1/refreshrate
         startsimtm = d.d[].time
         starttm = time()
         refreshtm = 1.0/(factor*s.refreshrate)
         updates = refreshtm / m.m[].opt.timestep

         steps = round(Int, round(s.framecount+updates)-s.framecount)
         s.framecount += updates

         for i=1:steps
            # clear old perturbations, apply new
            d.xfrc_applied .= 0.0

            # add in noise like perturbations
            # d.xfrc_applied[7:9] .= [5, 5, -10]
            # d.xfrc_applied[7:9] .= 10*randn(Float64, 3)
            
            if s.pert[].select > 0
               mjv_applyPerturbPose(m, d, s.pert, 0) # move mocap bodies only
               mjv_applyPerturbForce(m, d, s.pert)
            end

            t = d.d[].time

            # lower level update loop (eg state estimation, torque updates)
            if simulationstep % simulationsteps_per_controlstep == 0
               stepcontroller!(controller)
               s.d.ctrl .= controller.jointangles.data
               println(controller.jointangles[2, 1])
            end

            mj_step(s.m, s.d)
            simulationstep += 1

            # break on reset
            (d.d[].time < startsimtm) && break
         end
      end

      render(s, s.window)
      GLFW.PollEvents()
   end
   GLFW.DestroyWindow(s.window)
end

end
