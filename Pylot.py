import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import graphics
import aircraft
import simulation


class Panel:
    def __init__(self, screen_width=None):
        self.screen_width = screen_width
        glClearColor(0.,0.,0.,0.0)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        self.pic = pygame.image.load("V-n diagram.png")

    def draw(self):
        pic = self.pic.copy()

        if self.screen_width:
            pic = pygame.transform.scale(pic, (600,400))
        picData = pygame.image.tostring(pic,"RGBA",True)

        position = (-0.4,0.25,0)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA)
        glRasterPos3d(*position)
        glDrawPixels(pic.get_width(), pic.get_height(),GL_RGBA, GL_UNSIGNED_BYTE, picData)
        glDisable(GL_BLEND)


def main():
    #initialize pygame module and set window
    pygame.init()

    #DISPLAY WINDOW SIZE. CHANGE TO SUIT YOUR SCREEN IF NECESSARY
    width, height = 1680, 1050  # Desktop
#    width, height = 1366, 768  # Laptop
    pygame.display.set_icon(pygame.image.load('res/gameicon.jpg'))
    screen = pygame.display.set_mode((width,height), HWSURFACE|OPENGL|DOUBLEBUF)
    pygame.display.set_caption("Pylot")
    glViewport(0,0,width,height)
    glEnable(GL_DEPTH_TEST)
    default_view = np.identity(4)

    #boolean variables for camera view, lose screen, and pause
    FPV = True
    LOSE = False
    PAUSE = False
    KEYBOARD = False
    DATA = True

    #SIMULATION FRAMERATE
    #pygame limits framerate to this value. Increasing framerate improves accuracy of simulation (max limit = 60) but may cause some lag in graphics
    target_framerate = 60

    #initialize graphics objects
    #loading screen is rendered and displayed while the rest of the objects are read and prepared
    glClearColor(0.,0.,0.,1.0)
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
    loading = graphics.Text(150)
    loading.draw(-0.2,-0.05,"Loading...",(0,255,0,1))
    pygame.display.flip()

    #initialize game over screen
    gameover = graphics.Text(150)

    #initialize graphics aircraft
    graphics_aircraft = graphics.Mesh("res/Cessna.obj","shaders/aircraft.vs","shaders/aircraft.fs","res/cessna_texture.jpg",width,height)
    graphics_aircraft.set_orientation([0.,0.,0.,0.])
    graphics_aircraft.set_position([0.,0.,-500.])

    #initialize HUD
    HUD = graphics.HeadsUp(width, height)

    #initialize flight data overlay
    data = graphics.FlightData()

    #initialize field
    field = graphics.Mesh("res/field.obj","shaders/field.vs","shaders/field.fs","res/field_texture.jpg",width,height)
    field.set_position(np.array([0.,0.,0.]))

    #initialize camera object
    cam = graphics.Camera()

    a_obj = aircraft.Aircraft()
    dt = simulation.load_file('11.24_input.json', a_obj)
    simulation.initialize(a_obj)

    panel = Panel(width)

    #initialize other pygame elements
    if pygame.joystick.get_count()>0.:
        joy = pygame.joystick.Joystick(0)
        joy.init()
    else:
        KEYBOARD = True
        thr = 0.
        UP = False
        DOWN = False
        RIGHT = False
        LEFT = False
        WW = False
        SS = False
        AA = False
        DD = False

    #DEFLECTION LIMITS FOR CONTROL SURFACES
    d_ail = np.deg2rad(15.)
    d_ele = np.deg2rad(15.)
    d_rud = np.deg2rad(15.)

    #clock object for tracking frames and timestep
    clock = pygame.time.Clock()


    #ticks clock before starting game loop
    clock.tick_busy_loop()

    #game loop
    while True:
        #event loop checks for game inputs such as joystick and keyboard commands
        for event in pygame.event.get():
            if event.type == QUIT:
                return

            if KEYBOARD == False:
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == 2:
                        FPV = not FPV
                        cam.pos_storage.clear()
                        cam.up_storage.clear()
                        cam.target_storage.clear()
                    if event.button == 7:
                        PAUSE = not PAUSE
                    if event.button == 5:
                        a_obj.de_o += np.deg2rad(0.5)
                    if event.button == 4:
                        a_obj.de_o -= np.deg2rad(0.5)
            else:
                if event.type == pygame.KEYDOWN:

                    if event.key == pygame.K_UP:
                        UP = True
                    if event.key == pygame.K_DOWN:
                        DOWN = True
                    if event.key == pygame.K_LEFT:
                        LEFT = True
                    if event.key == pygame.K_RIGHT:
                        RIGHT = True

                    if event.key == pygame.K_w:
                        WW = True
                    if event.key == pygame.K_s:
                        SS = True
                    if event.key == pygame.K_a:
                        AA = True
                    if event.key == pygame.K_d:
                        DD = True
                    if event.key == pygame.K_SPACE:
                        FPV = not FPV
                        cam.pos_storage.clear()
                        cam.up_storage.clear()
                        cam.target_storage.clear()

                if event.type == pygame.KEYUP:
                    if event.key == pygame.K_UP:
                        UP = False
                    if event.key == pygame.K_DOWN:
                        DOWN = False
                    if event.key == pygame.K_LEFT:
                        LEFT = False
                    if event.key == pygame.K_RIGHT:
                        RIGHT = False

                    if event.key == pygame.K_w:
                        WW = False
                    if event.key == pygame.K_s:
                        SS = False
                    if event.key == pygame.K_a:
                        AA = False
                    if event.key == pygame.K_d:
                        DD = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_i:
                    DATA = not DATA
                #pause simulation
                if event.key == pygame.K_p:
                    PAUSE = not PAUSE
                #quit game
                if event.key == pygame.K_q:
                    return

        #maintains framerate even if sim is paused
        if PAUSE == True:
            clock.tick(target_framerate)

        #if game is not paused, runs sim
        if PAUSE == False:
            #set default background color for sky
            glClearColor(0.65,1.0,1.0,1.0)
            glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

            #timestep for simulation is based on framerate
            t= clock.tick(target_framerate)/1000.

            # if joystick is being used, gets joystick input and creates control_state dicitonary
            if KEYBOARD == False:
                a_obj.controls = [(joy.get_axis(4)**3)*-d_ele + a_obj.de_o,
                                  (joy.get_axis(3)**3)*-d_ail,
                                  (joy.get_axis(0)**3)*-d_rud,
                                  (-joy.get_axis(1)+1.)*0.5]


            # if joystick is not being used, gets keyboard input and creates control_state dictionary
            else:
                if UP == True and DOWN == False:
                    ele = 1.
                elif UP == False and DOWN == True:
                    ele = -1.
                else:
                    ele = 0.
                if LEFT == True and RIGHT == False:
                    ail = 1.
                elif LEFT == False and RIGHT == True:
                    ail = -1.
                else:
                    ail = 0.
                if AA == True and DD == False:
                    rud = 1.
                elif AA == False and DD == True:
                    rud = -1.
                else:
                    rud = 0.
                if WW == True and SS == False and thr<=1.0:
                    thr += 0.05
                elif WW == False and SS == True and thr>=0.0:
                    thr -= 0.05

            control_state = {
                "aileron": np.rad2deg(a_obj.controls[1]),
                "elevator": np.rad2deg(a_obj.controls[0]),
                "rudder": np.rad2deg(a_obj.controls[2]),
                "throttle": a_obj.controls[3],
                "flaps": 0.
                }

            #SIMULATION CALCULATIONS GO BELOW HERE FOR EACH TIME STEP
            #IT IS RECOMMENDED THAT YOU MAKE AN OBJECT FOR THE SIMULATION AIRCRAFT AND CREATE A FUNCTION IN SAID OBJECT TO CALCULATE THE NEXT TIME STEP.
            #THIS FUNCTION CAN THEN BE CALLED HERE
            simulation.run_sim(a_obj, dt)




            #INPUT POSITION, ORIENTATION, AND VELOCITY OF AIRCRAFT INTO THIS DICTIONARY WHICH WILL THEN UPDATE THE GRAPHICS
            aircraft_condition = {
                "Position":a_obj.state_vars[6:9],#input position of form [x,y,z]
                "Orientation":a_obj.state_vars[-4:],#input orientation in quaternion form [e0,ex,ey,ez]
                "Velocity":a_obj.state_vars[:3] #input Velocity of form [u,v,w]
            }
            flight_data = {
                "Graphics Time Step": t,#sec
                "Physics Time Step": dt,#sec
                "Airspeed": a_obj.V_now,#feet/sec
                "AoA":180.0*a_obj.alpha_now/np.pi ,#deg
                "Sideslip": a_obj.beta_now ,#deg
                "Altitude":-a_obj.state_vars[8] ,#feet
                "Latitude":a_obj.state_vars[6] ,#deg
                "Longitude":a_obj.state_vars[7] ,#deg
                "Time":0. ,#sec
                "Bank":a_obj.bank ,#deg
                "Elevation":a_obj.elevation ,#deg
                "Heading":a_obj.heading ,#deg
                "Gnd Speed":a_obj.V_now ,#feet/sec
                "Gnd Track":0. ,#deg
                "Climb":a_obj.climb, #feet/min
                "Throttle":control_state["throttle"]*100 ,#%
                "Elevator":control_state["elevator"] ,#deg
                "Ailerons":control_state["aileron"] ,#deg
                "Rudder":control_state["rudder"] ,#deg
                "Flaps":0.,#deg
                "Axial G-Force":0. ,#g's
                "Side G-Force":0. ,#g's
                "Normal G-Force":0. ,#g's
                "Roll Rate": a_obj.p_o ,#deg/s
                "Pitch Rate":a_obj.q_o ,#deg/s
                "Yaw Rate":a_obj.r_o #deg/s
            }

            #apply position and orientation to graphics
            graphics_aircraft.set_orientation(graphics.swap_quat(aircraft_condition["Orientation"]))
            graphics_aircraft.set_position(aircraft_condition["Position"])


            #test game over conditions
            if graphics_aircraft.position[2]>0.:
                LOSE = True


            #if you get a game over, display lose screen
            if LOSE == True:
                glClearColor(0,0,0,1.0)
                glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)

                gameover.draw(-0.2,-0.05,"Game Over",(0,255,0,1))
                PAUSE = True

            #otherwise, render graphics
            #Third person view
            elif FPV == False:
                #get view matrix and render scene
                view = cam.third_view(graphics_aircraft)
                graphics_aircraft.set_view(view)
                field.set_view(view)

                graphics_aircraft.render()
                field.render()
                panel.draw()
                if DATA == True:
                    data.render(flight_data)



            #cockpit view
            elif FPV == True:

                view = cam.cockpit_view(graphics_aircraft)
                field.set_view(view)

                field.render()


                if DATA == True:
                    data.render(flight_data)
                HUD.render(aircraft_condition,view)





            #update screen display
            pygame.display.flip()


if __name__ == "__main__":
    main()


