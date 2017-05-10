from gopigo import*
import time
import logging

class Fresh:
    def __init__(self):
        # LOG_LEVEL = logging.INFO
        LOG_LEVEL = logging.DEBUG
        LOG_FILE = "/home/pi/PnR-Final/log_robot.log"
        LOG_FORMAT = "%(asctime)s %(levelname)s %(message)s"
        logging.basicConfig(filename=LOG_FILE, format=LOG_FORMAT, level=LOG_LEVEL)
        print("\n ----This better work!\n")
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
        self.nav()

    def nav(self):
        logging.debug("Starting the nav method")
        print("\n --------STARTING NAVIGATION \n")
        counter = 0
        while True:
            self.stop()
            if counter == 3:
                logging.info("Restore heading, count at:" + str(count))
                self.restore_heading()
                counter = 0
            if self.is_clear():
                print("Looks clear, going forward")
                logging.debug("going forward in nav")
                fwd()
                while self.safe_driving():
                    time.sleep(.2)
                self.stop()
            answer = self.choose_path()
            if answer == "left":
                self.encL(5)
            else:
                self.encR(5)

    def safe_driving(self):
        if self.dist() > self.STOP_DIST:
            return True
        else:
            if self.dist() > self.STOP_DIST:
                return True
            return False

    def restore_heading(self):
        print("Now I'll turn back to the starting position.")
        self.set_speed(90, 90)
        if self.turn_track > 0:
            self.encL(abs(self.turn_track))
        elif self.turn_track < 0:
            self.encR(abs(self.turn_track))
        self.set_speed(self.LEFT_SPEED, self.RIGHT_SPEED)

    def set_speed(self, left, right):
        set_left_speed(left)
        set_right_speed(right)
        print('Left speed set to: ' + str(left) + ' // Right set to: ' + str(right))

    def encF(self, enc):
        print('Moving ' + str((enc / 18)) + ' rotation(s) forward')
        enc_tgt(1, 1, enc)
        fwd()
        time.sleep(1 * (enc / 18) + .5)

    def encR(self, enc):
        print('Moving ' + str((enc / 18)) + ' rotation(s) right')
        enc_tgt(1, 1, enc)
        right_rot()
        time.sleep(1 * (enc / 18) + .5)
        self.turn_track += enc

    def encL(self, enc):
        print('Moving ' + str((enc / 18)) + ' rotation(s) left')
        enc_tgt(1, 1, enc)
        left_rot()
        time.sleep(1 * (enc / 18) + .5)
        self.turn_track -= enc

    def encB(self, enc):
        print('Moving ' + str((enc / 18)) + ' rotations(s) backwards')
        enc_tgt(1, 1, enc)
        bwd()
        time.sleep(1 * (enc / 18) + .5)

    def servo(self, val):
        print('Moving servo to ' + str(val) + 'deg')
        servo(val)
        time.sleep(.1)

    def dist(self):
        measurement = us_dist(12)
        time.sleep(.05)
        print('I can see something ' + str(measurement) + "cm away")
        return measurement

    # DUMP ALL VALUES IN THE SCAN ARRAY
    def flush_scan(self):
        self.scan = [None] * 180

    # SEARCH 120 DEGREES COUNTING BY 2's
    def wide_scan(self):
        logging.debug("starting wide scan")
        self.flush_scan()
        self.stop()
        for x in range(self.MIDPOINT - 60, self.MIDPOINT + 60, +2):
            servo(x)
            time.sleep(.1)
            scan1 = us_dist(15)
            time.sleep(.1)
            # double check the distance
            scan2 = us_dist(15)
            # if I found a different distance the second time....
            if abs(scan1 - scan2) > 2:
                scan3 = us_dist(15)
                time.sleep(.1)
                # take another scan and average the three together
                scan1 = (scan1 + scan2 + scan3) / 3
            self.scan[x] = scan1
            print("Degree: " + str(x) + ", distance: " + str(scan1))
            time.sleep(.01)

    def is_clear(self):
        logging.debug("starting the is clear method")
        self.stop()
        print("Running the is_clear method.")
        for x in range((self.MIDPOINT - 20), (self.MIDPOINT + 20), 7):
            servo(x)
            time.sleep(.01)
            scan1 = us_dist(15)
            time.sleep(.01)
            # double check the distance
            scan2 = us_dist(15)
            time.sleep(.01)
            # if I found a different distance the second time....
            if abs(scan1 - scan2) > 2:
                scan3 = us_dist(15)
                time.sleep(.01)
                # take another scan and average the three together
                scan1 = (scan1 + scan2 + scan3) / 3
            self.scan[x] = scan1
            print("Degree: " + str(x) + ", distance: " + str(scan1))
            if scan1 < self.STOP_DIST:
                print("Doesn't look clear to me")
                logging.info("returning at the end of is clear with false")
                return False
        logging.info("looks clear my next move in is clear with true")
        return True

    # DECIDE WHICH WAY TO TURN
    def choose_path(self):
        logging.debug("starting the choose path method")
        print('Considering options...')
        self.stop()
        if self.is_clear():
            return "fwd"
        else:
            self.wide_scan()
        avgRight = 0
        avgLeft = 0
        for x in range(self.MIDPOINT - 60, self.MIDPOINT):
            if self.scan[x]:
                avgRight += self.scan[x]
        avgRight /= 60
        print('The average dist on the right is ' + str(avgRight) + 'cm')
        for x in range(self.MIDPOINT, self.MIDPOINT + 60):
            if self.scan[x]:
                avgLeft += self.scan[x]
        avgLeft /= 60
        print('The average dist on the left is ' + str(avgLeft) + 'cm')
        if avgRight > avgLeft:
            return "right"
        else:
            return "left"

    def stop(self):
        logging.debug("stop command received")
        print('All stop.')
        for x in range(5):
            stop()
            time.sleep(.01)

try:
    f = Fresh()
except (KeyboardInterrupt, SystemExit):
    from gopigo import*
    stop()
except Exception as ee:
    logging.error(ee.__str__())

