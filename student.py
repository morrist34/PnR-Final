import pigo
import time
import random

'''
MR. A's Final Project Student Helper
'''

class GoPiggy(pigo.Pigo):

    ########################
    ### CONTSTRUCTOR - this special method auto-runs when we instantiate a class
    #### (your constructor lasted about 9 months)
    ########################

    def __init__(self):
        print("Your piggy has be instantiated!")
        # Our servo turns the sensor. What angle of the servo( ) method sets it straight?
        self.MIDPOINT = 82
        # YOU DECIDE: How close can an object get (cm) before we have to stop?
        self.STOP_DIST = 30
        # YOU DECIDE: What left motor power helps straighten your fwd()?
        self.LEFT_SPEED = 70
        # YOU DECIDE: What left motor power helps straighten your fwd()?
        self.RIGHT_SPEED = 65
        # This one isn't capitalized because it changes during runtime, the others don't
        self.turn_track = 0
        # Our scan list! The index will be the degree and it will store distance
        self.scan = [None] * 180
        self.set_speed(self.LEFT_SPEED, self.RIGHT_SPEED)
        # let's use an event-driven model, make a handler of sorts to listen for "events"
        while True:
            self.stop()
            self.menu()


    ########################
    ### CLASS METHODS - these are the actions that your object can run
    #### (they can take parameters can return stuff to you, too)
    #### (they all take self as a param because they're not static methods)
    ########################


    ##### DISPLAY THE MENU, CALL METHODS BASED ON RESPONSE
    def menu(self):
        ## This is a DICTIONARY, it's a list with custom index values
        # You may change the menu if you'd like to add an experimental method
        menu = {"n": ("Navigate forward", self.final),
                "d": ("Dance", self.dance),
                "c": ("Calibrate", self.calibrate),
                "o": ("Count Obstacles", self.count_obstacles),
                "t": ("Turn Test", self.turn_test),
                "a": ("Count all obstacles", self.count_all_obstacles),
                "s": ("Check status", self.status),
                "q": ("Quit", quit)
                }
        # loop and print the menu...
        for key in sorted(menu.keys()):
            print(key + ":" + menu[key][0])
        # store the user's answer
        ans = raw_input("Your selection: ")
        # activate the item selected
        menu.get(ans, [None, error])[1]()

    def count_all_obstacles(self):
        big_counter = 0
        big_counter += self.count_obstacles()
        for x in range(4):
            self.encR(9)
            big_counter += self.count_obstacles()
        print('Total obstacles' + str(big_counter))
        return big_counter

    def count_obstacles(self):
        # run a scan
        self.wide_scan()
        #count how many obstacles I've found
        counter = 0
        #starting state assumes no obstacle
        found_something = False
        # loop thru all my scan data
        for x in self.scan:
            # if x is not none and really close
            if x and x <= self.STOP_DIST:
                #if ive already found something
                if found_something:
                    print("obstacle continues")
                    #if this is a new obstacle
                else:
                    # switch my tracker
                    found_something = True
                    print("start of new obstacle")
            #if my data show safe distances...
            if x and x > self.STOP_DIST:
                # if my tracker had been triggered
                if found_something:
                    print("end of obstacle")
                    # reset tracker
                    found_something = False
                    # increase count of obstacles
                    counter += 1
        print('total number of obstacles in this scan:' + str(counter))
        return counter


    def turn_test(self):
        while True:
            ans = raw_input('Turn right, left or stop? (r/l/s): ')
            if ans == 'r':
                val = int(raw_input('/nBy how much?: '))
                self.encR(val)
            elif ans == 'l':
                val = int(raw_input('/nBy how much?: '))
                self.encL(val)
            else:
                break
        self.restore_heading()

    def restore_heading(self):
        print("Now I'll turn back to the starting position.")
        self.set_speed(90,90)
        if self.turn_track > 0:
            self.encL(abs(self.turn_track))
        elif self.turn_track < 0:
            self.encR(abs(self.turn_track))
        self.set_speed(self.LEFT_SPEED, self.RIGHT_SPEED)
        '''
    def sweep(self):
        for x in range(self.MIDPOINT - 60, self.MIDPOINT + 60, 2):
            self.servo(x)
            self.scan[x] = self.dist()
        print("Here's what I saw: ")
        '''
    def safety_dance(self):
        for y in range(3):
            for x in range(self.MIDPOINT - 60, self.MIDPOINT + 60, 2):
                self.servo(x)
                if self.dist() < self.STOP_DIST:
                    print("AHHHH")
                    return
            self.encR(7)
        self.dance()


        #YOU DECIDE: How does your GoPiggy dance?
    def dance(self):
        print("Piggy dance")
        ##### WRITE YOUR FIRST PROJECT HERE
        self.shimmy()
        self.chacha()
        self.sprinkler()

    def shimmy(self):
        print('shimmy')
        for y in range(3):
            for x in range(150,250,50):
                self.set_speed(x,x)
            self.servo(40)
            self.encR(5)
            self.servo(140)
            self.encL(5)
            self.servo(40)
            self.encR(5)
            self.servo(40)
            self.encL(5)
    #example of unorganized code that should have been put in a loop
    def chacha(self):
        print('chacha')
        for x in range(2):
            self.servo(0)
            self.servo(45)
            self.encR(20)
            self.servo(45)
            self.encF(10)
            self.encB(10)
            self.encF(10)
            self.encB(10)
            self.encL(20)
            self.servo(30)
            self.encR(30)
            self.servo(30)
            self.encL(30)
            self.servo(30)
            self.encF(30)
            self.servo(30)
            self.encB(30)
            self.encF(10)
            self.encF(10)
            self.encR(20)
            self.servo(30)
            self.encR(20)
            self.servo(30)
    #example of organized code that is easy to follow
    def sprinkler(self):
        print('sprinkler')
        for x in range(160,20,15):
            self.servo(x)


    ########################
    ### MAIN LOGIC LOOP - the core algorithm of my navigation
    ### (kind of a big deal)
    ########################
    '''
    def nav(self):
        print("-----------! NAVIGATION ACTIVATED !------------\n")
        print("[ Press CTRL + C to stop me, then run stop.py ]\n")
        print("-----------! NAVIGATION ACTIVATED !------------\n")
        # this is the loop part of the "main logic loop"
        count = 0
        # see if i can use count_obstacles to nav easier
        '''

    def final(self):
        while True:
            if self.is_clear():
                self.cruise()
                self.wide_scan()
            answer = self.choose_path()
            if answer == "left":
                self.encL(4)
                self.fwd()
            elif answer == "right":
                self.encR(4)
                self.fwd()

    def cruise(self):
        self.sweep()
        self.fwd()
        while self.dist() > self.STOP_DIST:
            time.sleep(.15)
        self.stop()
        self.encB(3)

    def sweep(self):
        for x in range(self.MIDPOINT - 60, self.MIDPOINT + 60, 2):
            self.servo(x)
            self.scan[x] = self.dist()
        print("Here's what I saw")
        print(self.scan)

    def encR(self, enc):
        pigo.Pigo.encR(self, enc)
        self.turn_track += enc

    def encL(self, enc):
        pigo.Pigo.encL(self, enc)
        self.turn_track -= enc


####################################################
############### STATIC FUNCTIONS

def error():
    print('Error in input')


def quit():
    raise SystemExit

##################################################################
######## The app starts right here when we instantiate our GoPiggy

g = GoPiggy()
