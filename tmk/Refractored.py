#!/usr/bin/env python3

"""
Program za vodenje robota EV3
[Robo liga FRI 2019: Sadovnjak]
@Copyright: TrijeMaliKlinci & nejci
"""

from ev3dev.ev3 import TouchSensor, Button, LargeMotor, MediumMotor, Sound
import math
from time import time, sleep
from collections import deque
from io import BytesIO
import pycurl
import ujson
from enum import Enum
import sys


class State(Enum):
    """
    Stanja robota.
    """

    def __str__(self):
        return str(self.name)

    GET_APPLE = 0
    GET_TURN = 1
    GET_STRAIGHT = 2
    HOME = 3
    HOME_TURN = 4
    HOME_STRAIGHT = 5
    BACK_OFF = 6
    ENEMY_HOME = 7
    ENEMY_HOME_TURN = 8
    ENEMY_HOME_STRAIGHT = 9
    GET_BAD_APPLE = 10
    CLEAR_HOME = 11
    CLEAR_TURN = 12
    CLEAR_STRAIGHT = 13
    CLEAR_OUT = 14


# from tmk.classes.Pid import PID


class PID:
    def __init__(
            self,
            setpoint: float,
            kp: float,
            ki: float = None,
            kd: float = None,
            integral_limit: float = None):
        """
        Ustvarimo nov regulator PID s pripadajočimi parametri.

        Argumenti:
        setpoint: ciljna vrednost regulirane spremenljivke
        kp: ojačitev proporcionalnega dela regulatorja.
            Visoke vrednosti pomenijo hitrejši odziv sistema,
            vendar previsoke vrednosti povzročijo oscilacije in nestabilnost.
        ki: ojačitev integralnega člena regulatorja.
            Izniči napako v ustaljenem stanju. Zmanjša odzivnost.
        kd: ojačitev odvoda napake.
            Zmanjša čas umirjanja in poveča odzivnost.
        integral_limit: najvišja vrednost integrala
        """
        self._setpoint = setpoint
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._integral_limit = integral_limit
        self._error = None
        self._time = None
        self._integral = None
        self._value = None

    def reset(
            self,
            setpoint: float = None,
            kp: float = None,
            ki: float = None,
            kd: float = None,
            integral_limit: float = None):
        """
        Ponastavitev regulatorja.
        Lahko mu tudi spremenimo katero od vrednosti parametrov.
        Napaka, integral napake in čas se ponastavijo.
        """
        if setpoint is not None:
            self._setpoint = setpoint
        if kp is not None:
            self._kp = kp
        if ki is not None:
            self._ki = ki
        if kd is not None:
            self._kd = kd
        if integral_limit is not None:
            self._integral_limit = integral_limit
        self._error = None
        self._time = None
        self._integral = None
        self._value = None

    def update(self, measurement: float) -> float:
        """
        Izračunamo vrednost izhoda regulatorja (regulirna veličina)
        glede na izmerjeno vrednost regulirane veličine (measurement)
        in ciljno vrednost (setpoint).

        Argumenti:
        measurement: s tipali izmerjena vrednost regulirane veličine

        Izhodna vrednost:
        regulirna veličina, s katero želimo popraviti delovanje sistema
        (regulirano veličino), da bo dosegel ciljno vrednost
        """
        if self._value is None:
            # Na začetku še nimamo zgodovine meritev, zato inicializiramo
            # integral in vrnemo samo proporcionalni člen.
            self._value = measurement
            # Zapomnimo si začetni čas.
            self._time = time()
            # Ponastavimo integral napake.
            self._integral = 0
            # Napaka = ciljna vrednost - izmerjena vrednost regulirane veličine.
            self._error = self._setpoint - measurement
            return self._kp * self._error
        else:
            # Sprememba časa
            time_now_pid = time()
            delta_time = time_now_pid - self._time
            self._time = time_now_pid
            # Izmerjena vrednost regulirane veličine.
            self._value = measurement
            # Napaka = ciljna vrednost - izmerjena vrednost regulirane veličine.
            error = self._setpoint - self._value

            # Proporcionalni del
            p = self._kp * error

            # Integralni in odvodni člen sta opcijska.
            if self._ki is None:
                i = 0
            else:
                # Integral se poveča za (sprememba napake) / (sprememba časa).
                self._integral += error * delta_time
                # Ojačitev integralnega dela.
                i = self._ki * self._integral
                if self._integral_limit is not None:
                    i = max(min(i, self._integral_limit),
                            (-1) * self._integral_limit)

            if self._kd is None:
                d = 0
            else:
                # Odvod napake z ojačitvijo.
                d = self._kd * (error - self._error) / delta_time
            # Posodobimo napako.
            self._error = error
            # Vrnemo regulirno veličino, sestavljeno iz proporcionalnega,
            # integralnega in odvodnega člena.
            return p + i + d


# from tmk.classes.Point import Point


class Point:
    """
    Točka na poligonu.
    """

    def __init__(self, position):
        self.x = position[0]
        self.y = position[1]

    def __str__(self):
        return '(' + str(self.x) + ', ' + str(self.y) + ')'


# from tmk.classes.Connection import Connection


class Connection:
    """
    Objekt za vzpostavljanje povezave s strežnikom.
    """

    def __init__(self, url_connection: str):
        """
        Inicializacija nove povezave.

        Argumenti:
        url_connection: pot do datoteke na strežniku (URL)
        """
        self._url = url_connection
        self._buffer = BytesIO()
        self._pycurlObj = pycurl.Curl()
        self._pycurlObj.setopt(self._pycurlObj.URL, self._url)
        self._pycurlObj.setopt(self._pycurlObj.CONNECTTIMEOUT, 10)
        self._pycurlObj.setopt(self._pycurlObj.WRITEDATA, self._buffer)

    def request(self, debug=False):
        """
        Nalaganje podatkov s strežnika.
        """
        # Počistimo pomnilnik za shranjevanje sporočila
        self._buffer.seek(0, 0)
        self._buffer.truncate()
        # Pošljemo zahtevek na strežnik
        self._pycurlObj.perform()
        # Dekodiramo sporočilo
        msg = self._buffer.getvalue().decode()
        # Izluščimo podatke iz JSON
        try:
            return ujson.loads(msg)
        except ValueError as val_err:
            if debug:
                print('Napaka pri razclenjevanju datoteke JSON: ' + str(val_err))
                print('Sporocilo streznika:')
                print(msg)
            return -1

    def test_delay(self, num_iters: int = 10):
        """
        Merjenje zakasnitve pri pridobivanju podatkov o tekmi s strežnika.
        Zgolj informativno.
        """
        sum_time = 0
        for i in range(num_iters):
            start_time = time()
            if self.request() == -1:
                robot_die()
            elapsed_time = time() - start_time
            sum_time += elapsed_time
        return sum_time / num_iters


# -----------------------------------------------------------------------
# INITIALIZATION FUNCTIONS and OTHERS


def get_angle(p1, a1, p2) -> float:
    """
    Izračunaj kot, za katerega se mora zavrteti robot, da bo obrnjen proti točki p2.
    Robot se nahaja v točki p1 in ima smer (kot) a1.
    """
    a = math.degrees(math.atan2(p2.y - p1.y, p2.x - p1.x))
    a_rel = a - a1
    if abs(a_rel) > 180:
        if a_rel > 0:
            a_rel = a_rel - 360
        else:
            a_rel = a_rel + 360

    return a_rel


def get_distance(p1: Point, p2: Point) -> float:
    """
    Evklidska razdalja med dvema točkama na poligonu.
    """
    return math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)


def get_distance_from_apple_to_robot(apple) -> float:
    return get_distance(get_apple_pos(apple), robot_pos)


def init_large_motor(port: str) -> LargeMotor:
    """
    Preveri, ali je motor priklopljen na izhod `port`.
    Vrne objekt za motor (LargeMotor).
    """
    motor = LargeMotor(port)
    while not motor.connected:
        print('\nPriklopi motor na izhod ' + port +
              ' in pritisni + spusti gumb DOL.')
        wait_for_button('down')
        motor = LargeMotor(port)
    return motor


def init_medium_motor(port: str) -> MediumMotor:
    """
    Preveri, ali je motor priklopljen na izhod `port`.
    Vrne objekt za motor (LargeMotor).
    """
    motor = MediumMotor(port)
    while not motor.connected:
        print('\nPriklopi motor na izhod ' + port +
              ' in pritisni + spusti gumb DOL.')
        wait_for_button('down')
        motor = MediumMotor(port)
    return motor


def init_sensor_touch() -> TouchSensor:
    """
    Preveri, ali je tipalo za dotik priklopljeno na katerikoli vhod.
    Vrne objekt za tipalo.
    """
    sensor = TouchSensor()
    while not sensor.connected:
        print('\nPriklopi tipalo za dotik in pritisni + spusti gumb DOL.')
        wait_for_button('down')
        sensor = TouchSensor()
    return sensor


def wait_for_button(btn_name: str = 'down'):
    """
    Čakaj v zanki dokler ni gumb z imenom `btn_name` pritisnjen in nato sproščen.
    """
    while not getattr(btn, btn_name):
        pass
    flag = False
    while getattr(btn, btn_name):
        if not flag:
            flag = True


def beep(duration=1000, freq=440):
    """
    Potrobi s frekvenco `freq` za čas `duration`. Klic ne blokira.
    """
    Sound.tone(freq, duration)
    # Če želimo, da blokira, dokler se pisk ne konča.
    # Sound.tone(freq, duration).wait()


def robot_die():
    """
    Končaj s programom na robotu. Ustavi motorje.
    """
    print('KONEC')
    motor_left.stop(stop_action='brake')
    motor_right.stop(stop_action='brake')
    motor_grab.stop(stop_action='brake')
    Sound.play_song((
        ('D4', 'e'),
        ('C4', 'e'),
        ('A3', 'h')))
    sys.exit(0)


# -----------------------------------------------------------------------
# APPLE RELATED FUNCTIONS


def get_closest_good_apple():
    """
    Funkcija vrne najbližje zdravo jabolko brez upoštevanja jabolka z 'closest id'
    """
    min_apple = None
    min_dist = float("inf")
    for apple in game_state['apples']:
        if apple['type'] == "appleGood" and not at_home(get_apple_pos(apple)):
            atm_dist = get_distance(get_robot_pos(), Point(apple['position'][0:2]))
            if atm_dist < min_dist:
                min_dist = atm_dist
                min_apple = apple
    return min_apple


def get_closest_bad_apple():
    """
    Funkcija vrne najbližje gnilo jabolko
    """
    min_apple = None
    min_dist = float("inf")
    for apple in game_state['apples']:
        if apple['type'] == "appleBad" and not at_home_enemy(get_apple_pos(apple)):
            atm_dist = get_distance(get_robot_pos(), Point(apple['position'][0:2]))
            if atm_dist < min_dist:
                min_dist = atm_dist
                min_apple = apple
    return min_apple


def apple_in_claws(apple_id):
    apple = get_apple_by_id(apple_id)
    if apple is None:
        return False
    apple_posi = get_apple_pos(apple)
    # izmerjeno 13 cm
    new_point = point_transpose(get_robot_pos(), get_robot_dir(), 110)
    # print(str(new_point.x) + " " + str(new_point.y))
    x_low = new_point.x - 70
    x_high = new_point.x + 70
    y_low = new_point.y - 70
    y_high = new_point.y + 70

    if x_low < apple_posi.x < x_high and y_low < apple_posi.y < y_high:
        print("APPLE IS IN CLAWS")
        return True
    return False


def get_apple_by_id(apple_id):
    """
    Funkcija vrne "objekt" jabolka s id-jem 'apple_id'
    """
    for apple in game_state['apples']:
        if get_apple_id(apple) == apple_id:
            return apple
    return None


def get_apple_id(apple):
    return apple['id']


def get_apple_pos(apple):
    apple_id = get_apple_id(apple)
    apple = get_apple_by_id(apple_id)
    return Point(apple['position'])


def get_apple_type(apple):
    return apple['type']


# -----------------------------------------------------------------------
# GAME STATE GETTERS


def get_time_left():
    return game_state['timeLeft']


def get_team_one():
    return game_state['team1']


def get_team_two():
    return game_state['team2']


def get_baskets():
    return game_state['field']['baskets']


def get_apples():
    return game_state['apples']


def get_robots():
    return game_state['robots']


# ------------------------------------------------------------------------
# TEAM GETTERS


def get_team_score():
    return game_state[team_my_tag]['score']


def get_enemy_team_score():
    return game_state[team_op_tag]['score']


# ------------------------------------------------------------------------
# FIELD GETTERS


def get_top_left_corner() -> Point:
    return Point(get_baskets()['topLeft'])


def get_top_right_corner() -> Point:
    return Point(get_baskets()['topRight'])


def get_bottom_left_corner() -> Point:
    return Point(get_baskets()['bottomLeft'])


def get_bottom_right_corner() -> Point:
    return Point(get_baskets()['bottomRight'])


def get_basket_top_left_corner() -> Point:
    return Point(get_baskets()[team_my_tag]['topLeft'])


def get_basket_top_right_corner() -> Point:
    return Point(get_baskets()[team_my_tag]['topRight'])


def get_basket_bottom_left_corner() -> Point:
    return Point(get_baskets()[team_my_tag]['bottomLeft'])


def get_basket_bottom_right_corner() -> Point:
    return Point(get_baskets()[team_my_tag]['bottomRight'])


def get_basket_enemy_top_left_corner() -> Point:
    return Point(get_baskets()[team_op_tag]['topLeft'])


def get_basket_enemy_top_right_corner() -> Point:
    return Point(get_baskets()[team_op_tag]['topRight'])


def get_basket_enemy_bottom_left_corner() -> Point:
    return Point(get_baskets()[team_op_tag]['bottomLeft'])


def get_basket_enemy_bottom_right_corner() -> Point:
    return Point(get_baskets()[team_op_tag]['bottomRight'])


# ------------------------------------------------------------------------
# ROBOT GETTERS


def get_robot_pos() -> Point:
    """
    Funkcija vrne trenutno pozicijo robota
    """
    for robot_data_iter in game_state['robots']:
        if robot_data_iter['id'] == ROBOT_ID:
            return Point(robot_data_iter['position'][0:2])
    return None


def get_enemy_robot_pos() -> Point:
    """
    Funkcija vrne trenutno pozicijo nasprotnika
    """
    for robot_data_iter in game_state['robots']:
        if robot_data_iter['id'] != ROBOT_ID:
            return Point(robot_data_iter['position'][0:2])
    return None


def get_robot_dir():
    for robot_data_iter in game_state['robots']:
        if robot_data_iter['id'] == ROBOT_ID:
            return robot_data_iter['direction']
    return 0


def get_enemy_robot_dir():
    for robot_data_iter in game_state['robots']:
        if robot_data_iter['id'] != ROBOT_ID:
            return robot_data_iter['direction']
    return 0


# ------------------------------------------------------------------------
# MISCELLANEOUS FUNCTIONS


def claws_open():
    # https://www.ev3dev.org/docs/tutorials/tacho-motors/
    motor_right.run_forever(speed_sp=0)
    motor_left.run_forever(speed_sp=0)
    motor_grab.run_forever(speed_sp=1000)
    sleep(0.4)
    motor_grab.stop(stop_action='hold')


def claws_close():
    motor_right.run_forever(speed_sp=0)
    motor_left.run_forever(speed_sp=0)
    motor_grab.run_forever(speed_sp=-1000)
    sleep(0.4)
    motor_grab.stop(stop_action='hold')


def at_home(position: Point):
    if get_basket_top_left_corner().x < position.x < get_basket_top_right_corner().x:
        if get_basket_bottom_left_corner().y < position.y < get_basket_top_right_corner().y:
            return True
    return False


def at_home_enemy(position: Point):
    if get_basket_enemy_top_left_corner().x < position.x < get_basket_enemy_top_right_corner().x:
        if get_basket_enemy_bottom_left_corner().y < position.y < get_basket_enemy_top_right_corner().y:
            return True
    return False


def point_transpose(curr: Point, direction, length):
    if direction < 0:
        direction = -direction
    else:
        direction = 360 - direction

    curr.x += (math.cos(math.radians(direction))) * length
    curr.y -= (math.sin(math.radians(direction))) * length
    return curr


def decelerate_both_motors_to(curr_speed, wanted_speed):
    while curr_speed > wanted_speed:
        motor_right.run_forever(speed_sp=curr_speed)
        motor_left.run_forever(speed_sp=curr_speed)
        curr_speed -= 4
        sleep(0.001)


def apples_on_path(length, width):
    curr_pos = get_robot_pos()
    curr_dir = get_robot_dir()
    temp_var = Point([curr_pos.x, curr_pos.y])
    top_left = point_transpose(temp_var, curr_dir - 90, width)
    temp_var = Point([curr_pos.x, curr_pos.y])
    bottom_left = point_transpose(temp_var, curr_dir + 90, width)
    temp_var = Point([bottom_left.x, bottom_left.y])
    bottom_right = point_transpose(temp_var, curr_dir, length)

    on_path = []
    apples = get_apples()
    for apple in apples:
        if get_apple_id(apple) == get_apple_id(current_apple):
            continue
        apple_posi = get_apple_pos(apple)
        if top_left.x < apple_posi.x < bottom_right.x and top_left.y < apple_posi.y < bottom_right.y:
            on_path.append(apple)

    return on_path


def encoder_apple_in_claws():
    if encoder_closed - 10 < motor_grab.position < encoder_open + 10:
        return True
    return False


def is_point_on_map(point1: Point):
    if 0 < point1.x < get_bottom_right_corner().x and \
            0 < point1.y < get_top_left_corner().y:
        return True
    return False


def get_temp_home():
    offset = 50
    new_home = home
    if team_my_tag == "team1":
        x = get_basket_top_right_corner().x - offset
        y_top = get_basket_top_right_corner().y - offset
        y_bot = get_basket_bottom_right_corner().y + offset

        if robot_pos.x < x:
            new_home.x = robot_pos.x
        else:
            new_home.x = x

        if robot_pos.y < y_bot:
            new_home.y = y_bot
        elif robot_pos.y > y_top:
            new_home.y = y_top
        else:
            new_home.y = robot_pos.y

    else:
        x = get_basket_top_left_corner().x + offset
        y_top = get_basket_top_left_corner().y - offset
        y_bot = get_basket_bottom_left_corner().y + offset

        if robot_pos.x > x:
            new_home.x = robot_pos.x
        else:
            new_home.x = x

        if robot_pos.y < y_bot:
            new_home.y = y_bot
        elif robot_pos.y > y_top:
            new_home.y = y_top
        else:
            new_home.y = robot_pos.y

    return new_home


def bad_apples_at_home():
    apples = get_apples()
    res = []
    for apple in apples:
        if get_apple_type(apple) == "appleBad":
            if at_home(get_apple_pos(apple)):
                res.append(apple)

    return res


# ------------------------------------------------------------------------
# CONSTANTS
# ------------------------------------------------------------------------
# ID robota. Spremenite, da ustreza številki označbe, ki je določena vaši ekipi.
ROBOT_ID = 35
# Konfiguracija povezave na strežnik. LASPP strežnik ima naslov "192.168.0.3".
SERVER_IP = "192.168.0.153"
# Datoteka na strežniku s podatki o tekmi.
GAME_STATE_FILE = "game.json"

# Priklop motorjev na izhode.
MOTOR_LEFT_PORT = 'outA'
MOTOR_RIGHT_PORT = 'outD'
MOTOR_GRAB_PORT = 'outC'

# Najvišja dovoljena hitrost motorjev.
SPEED_MAX = 1000
# Najvišja dovoljena nazivna hitrost motorjev pri vožnji naravnost.
# Naj bo manjša kot SPEED_MAX, da ima robot še možnost zavijati.
SPEED_BASE_MAX = 900

# Parametri za PID
# Obračanje na mestu
PID_TURN_KP = 1.0  # 1.4
PID_TURN_KI = 0.0
PID_TURN_KD = 0.0  # 0.53
PID_TURN_INT_MAX = 100
# Nazivna hitrost pri vožnji naravnost.
PID_FRWD_KP = 1.0  # 1.0
PID_FRWD_KI = 0.0
PID_FRWD_KD = 0.0  # 0.1
PID_FRWD_INT_MAX = 100
# Zavijanje med vožnjo naravnost
PID_FRWD_TURN_KP = 3.0  # 15
PID_FRWD_TURN_KI = 0.0
PID_FRWD_TURN_KD = 0.0  # 2
PID_FRWD_TURN_INT_MAX = 100
# Obračanje na mestu S KOCKO
PID_TURN_APPLE_KP = 1.0  # 4.0, 3.0
PID_TURN_APPLE_KI = 0.0
PID_TURN_APPLE_KD = 0.05  # 0.05
PID_TURN_APPLE_INT_MAX = 100
# Nazivna hitrost pri vožnji naravnost S KOCKO
PID_FRWD_APPLE_KP = 1.0  # 5.0
PID_FRWD_APPLE_KI = 0.0
PID_FRWD_APPLE_KD = 0.0
PID_FRWD_APPLE_INT_MAX = 100
# Zavijanje med vožnjo naravnost s jabolkom
PID_FRWD_TURN_APPLE_KP = 15.0
PID_FRWD_TURN_APPLE_KI = 0.0
PID_FRWD_TURN_APPLE_KD = 0.0
PID_FRWD_TURN_APPLE_INT_MAX = 100

# Dolžina FIFO vrste za hranjenje meritev (oddaljenost in kot do cilja).
HIST_QUEUE_LENGTH = 3

# Razdalje - tolerance
# Dovoljena napaka v oddaljenosti do cilja [mm].
DIST_EPS = 100
# Dovoljena napaka pri obračanju [stopinje].
DIR_EPS = 5
# Bližina cilja [mm].
DIST_NEAR = 100
# Koliko sekund je robot lahko stanju vožnje naravnost v bližini cilja
# (oddaljen manj kot DIST_NEAR), preden sprožimo varnostni mehanizem
# in ga damo v stanje obračanja na mestu.
TIMER_NEAR_TARGET = 3

# -----------------------------------------------------------------------------
# NASTAVITVE TIPAL, MOTORJEV IN POVEZAVE S STREŽNIKOM
# -----------------------------------------------------------------------------
# Nastavimo tipala in gumbe.
print('Priprava tipal ... ', end='', flush=True)
btn = Button()
# sensor_touch = init_sensor_touch()
print('OK!')

# Nastavimo velika motorja. Priklopljena naj bosta na izhoda A in D.
print('Priprava motorjev ... ', end='')
motor_left = init_large_motor(MOTOR_LEFT_PORT)
motor_right = init_large_motor(MOTOR_RIGHT_PORT)
motor_grab = init_medium_motor(MOTOR_GRAB_PORT)
print('OK!')

claws_close()
encoder_closed = motor_grab.position
claws_open()
encoder_open = motor_grab.position

# Nastavimo povezavo s strežnikom.
url = SERVER_IP + '/' + GAME_STATE_FILE
print('Vspostavljanje povezave z naslovom ' + url + ' ... ', end='', flush=True)
conn = Connection(url)
print('OK!')

# Izmerimo zakasnitev pri pridobivanju podatkov (povprečje num_iters meritev)
print('Zakasnitev v komunikaciji s streznikom ... ', end='', flush=True)
print('%.4f s' % (conn.test_delay(num_iters=10)))

# -----------------------------------------------------------------------------
# PRIPRAVA NA TEKMO
# -----------------------------------------------------------------------------
# Pridobimo podatke o tekmi.
game_state = conn.request()
# Ali naš robot sploh tekmuje? Če tekmuje, ali je team1 ali team2?
team_my_tag = 'undefined'
team_op_tag = 'undefined'

if ROBOT_ID == get_team_one()['id']:
    team_my_tag = 'team1'
    team_op_tag = 'team2'
elif ROBOT_ID == get_team_two()['id']:
    team_my_tag = 'team2'
    team_op_tag = 'team1'
else:
    print('Robot ne tekmuje.')
    robot_die()
print('Robot tekmuje in ima interno oznako "' + team_my_tag + '"')

# -----------------------------------------------------------------------------
# PIDi
# -----------------------------------------------------------------------------

# Multiplier-ji
pid_frwd_base_multiplier = 1
pid_frwd_base_apple_multiplier = 1

# Regulator PID za obračanje na mestu.
# setpoint=0 pomeni, da naj bo kot med robotom in ciljem (target_angle) enak 0.
# Naša regulirana veličina je torej kar napaka kota, ki mora biti 0.
# To velja tudi za regulacijo vožnje naravnost.
PID_turn = PID(
    setpoint=0,
    kp=PID_TURN_KP,
    ki=PID_TURN_KI,
    kd=PID_TURN_KD,
    integral_limit=PID_TURN_INT_MAX)

# PID za vožnjo naravnost - regulira nazivno hitrost za oba motorja,
# ki je odvisna od oddaljenosti od cilja.
# setpoint=0 pomeni, da mora biti razdalja med robotom in ciljem enaka 0.
PID_frwd_base = PID(
    setpoint=0,
    kp=PID_FRWD_KP,
    ki=PID_FRWD_KI,
    kd=PID_FRWD_KD,
    integral_limit=PID_FRWD_INT_MAX)

# PID za obračanje med vožnjo naravnost.
# setpoint=0 pomeni, da naj bo kot med robotom in ciljem (target_angle) enak 0.
PID_frwd_turn = PID(
    setpoint=0,
    kp=PID_FRWD_TURN_KP,
    ki=PID_FRWD_TURN_KI,
    kd=PID_FRWD_TURN_KD,
    integral_limit=PID_FRWD_TURN_INT_MAX)

# PID za obračanje na mestu s jabolkom
PID_turn_apple = PID(
    setpoint=0,
    kp=PID_TURN_APPLE_KP,
    ki=PID_TURN_APPLE_KI,
    kd=PID_TURN_APPLE_KD,
    integral_limit=PID_TURN_APPLE_INT_MAX)

# PID za obračanje na mestu s jabolkom
PID_frwd_base_apple = PID(
    setpoint=0,
    kp=PID_FRWD_APPLE_KP,
    ki=PID_FRWD_APPLE_KI,
    kd=PID_FRWD_APPLE_KD,
    integral_limit=PID_FRWD_APPLE_INT_MAX)

# PID za obračanje na mestu s jabolkom
PID_frwd_turn_apple = PID(
    setpoint=0,
    kp=PID_FRWD_TURN_APPLE_KP,
    ki=PID_FRWD_TURN_APPLE_KI,
    kd=PID_FRWD_TURN_APPLE_KD,
    integral_limit=PID_FRWD_TURN_APPLE_INT_MAX)

# -----------------------------------------------------------------------------
# GLOBALNE SPREMENLJIVKE
# -----------------------------------------------------------------------------
# Nastavi točko za domov
home = get_basket_top_left_corner()
home.x += 270
home.y -= 515
# Nastavi točko za dom nasprotnika
enemy_home = get_basket_enemy_top_left_corner()
enemy_home.x += 270
enemy_home.y -= 515
# Hitrost na obeh motorjih.
speed_right = 0
speed_left = 0
# Stara hitrost
speed_right_old = 0
speed_left_old = 0
# Zgodovina (okno) zadnjih nekaj vrednosti meritev,
# implementirana kot vrsta FIFO.
robot_dir_hist = deque([180.0] * HIST_QUEUE_LENGTH)
robot_dist_hist = deque([math.inf] * HIST_QUEUE_LENGTH)
# Meritve direction
robot_dir_data_id = 0
# Merimo čas obhoda zanke. Za visoko odzivnost robota je zelo pomembno,
# da je ta čas čim krajši.
t_old = time()
# Začetno stanje.
state = State.GET_APPLE
# Prejšnje stanje.
state_old = -1
# Id prejšnjega najbližjega jabolka
picked_up_apples_id = []
# Trenutno jabolko
current_apple = None
# Trenutni target
target = None
# Razdalja med robotom in ciljem.
target_dist = 0
# Kot med robotom in ciljem.
target_angle = 0
# Datoteke za zapis podatkov za graf
file = open('pid_data' + str(robot_dir_data_id) + '.txt', 'w')
# Če se zatakne timer
time_timeout = 0
# Pospešek
get_straight_accel_factor = 0.05

# -----------------------------------------------------------------------------
# GLAVNA ZANKA
# -----------------------------------------------------------------------------
print('Izvajam glavno zanko. Prekini jo s pritiskon na tipko DOL.')
print('Cakam na zacetek tekme ...')

do_main_loop = True
while do_main_loop and not btn.down:

    time_now = time()
    loop_time = time_now - t_old
    t_old = time_now

    # Osveži stanje tekme.
    game_state = conn.request()
    if game_state == -1:
        print('Napaka v paketu, ponovni poskus ...')
    else:
        game_on = game_state['gameOn']
        time_left = get_time_left()

        # Pridobi pozicijo in orientacijo svojega robota;
        # najprej pa ga poišči v tabeli vseh robotov na poligonu.
        robot_pos = get_robot_pos()
        robot_dir = get_robot_dir()
        # Ali so podatki o robotu veljavni? Če niso, je zelo verjetno,
        # da sistem ne zazna oznake na robotu.
        robot_alive = (robot_pos is not None) and (robot_dir is not None)

        # Če tekma poteka in je oznaka robota vidna na kameri,
        # potem izračunamo novo hitrost na motorjih.
        # Sicer motorje ustavimo.
        if game_on and robot_alive:

            # Zaznaj spremembo stanja.
            if state != state_old:
                print(state.__str__())
                time_timeout = time()
                state_changed = True
            else:
                if time() - time_timeout > 8:
                    state = State.BACK_OFF
                state_changed = False
            state_old = state

            # Spremljaj zgodovino meritev kota in oddaljenosti.
            # Odstrani najstarejši element in dodaj novega - princip FIFO.
            robot_dir_hist.popleft()
            robot_dir_hist.append(target_angle)
            robot_dist_hist.popleft()
            robot_dist_hist.append(target_dist)

            if state == State.GET_APPLE:
                # Nastavi target na najbližje jabolko
                # print("State GET_APPLE")
                # if get_time_left() < 60:
                #    if bad_apples_at_home().__len__() > 0:
                #        state = State.CLEAR_HOME
                #        continue

                current_apple = get_closest_good_apple()
                if current_apple is None:
                    state = State.GET_BAD_APPLE
                    continue

                target = get_apple_pos(current_apple)
                # print(str(target.x) + " " + str(target.y))

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                speed_right = 0
                speed_left = 0

                # Preverimo, ali je robot na ciljni točki.
                # Če ni, ga tja pošljemo.
                if target_dist > DIST_EPS:
                    state = State.GET_TURN
                    robot_near_target_old = False
                else:
                    state = State.HOME

            elif state == State.GET_BAD_APPLE:
                # Nastavi target na najbližje jabolko
                # print("State GET_BAD_APPLE")

                current_apple = get_closest_bad_apple()
                if current_apple is None:
                    state = State.GET_APPLE
                    continue

                target = get_apple_pos(current_apple)
                # print(str(target.x) + " " + str(target.y))

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                speed_right = 0
                speed_left = 0

                # Preverimo, ali je robot na ciljni točki.
                # Če ni, ga tja pošljemo.
                if target_dist > DIST_EPS:
                    state = State.GET_TURN
                    robot_near_target_old = False
                else:
                    state = State.ENEMY_HOME

            elif state == State.HOME:
                # Nastavi target na home
                # print("State HOME")

                target = get_temp_home()
                print("Home coords: " + str(target.x) + " " + str(target.y))

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                speed_right = 0
                speed_left = 0

                # Preverimo, ali je robot na ciljni točki.
                # Če ni, ga tja pošljemo.
                if target_dist > DIST_EPS:
                    state = State.HOME_TURN
                    robot_near_target_old = False
                else:
                    state = State.GET_APPLE

            elif state == State.ENEMY_HOME:
                # Nastavi target na home
                # print("State ENEMY_HOME")

                target = enemy_home
                print("Target coords: " + str(target.x) + " " + str(target.y))

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                speed_right = 0
                speed_left = 0

                # Preverimo, ali je robot na ciljni točki.
                # Če ni, ga tja pošljemo.
                if target_dist > DIST_EPS:
                    state = State.ENEMY_HOME_TURN
                    robot_near_target_old = False
                else:
                    state = State.GET_APPLE

            elif state == State.GET_TURN:

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                # beleženje za izris grafa
                # file.write(str(target_angle) + ',' + str(time_now) + '\n')

                if state_changed:
                    # Če smo ravno prišli v to stanje, najprej ponastavimo PID.
                    PID_turn.reset()

                # Ali smo že dosegli ciljni kot?
                # Zadnjih nekaj obhodov zanke mora biti absolutna vrednost
                # napake kota manjša od DIR_EPS.
                err = [abs(a) > DIR_EPS for a in robot_dir_hist]

                if sum(err) == 0:
                    # Vse vrednosti so znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    state = State.GET_STRAIGHT

                else:
                    u = PID_turn.update(measurement=target_angle)
                    speed_right = -u
                    speed_left = u

            elif state == State.GET_STRAIGHT:
                # Vožnja robota naravnost proti ciljni točki.
                # print("State GET_STRAIGHT")

                # Predikcija kje se bomo nahajali v naslednji iteraciji
                # Če bi bili izven mape, gremo v stanje GET_TURN
                # Preverja 5 cm pred sabo
                if not is_point_on_map(point_transpose(robot_pos, robot_dir, 50)):
                    speed_left = 0
                    speed_right = 0
                    state = State.GET_TURN
                    continue

                # Poglej če je target sploh še tam
                apple_pos = get_apple_pos(current_apple)
                if not (target.x - 50 < apple_pos.x < target.x + 50 and target.y - 50 < apple_pos.y < target.y + 50):
                    speed_left = 0
                    speed_right = 0
                    state = State.GET_APPLE
                    continue

                # Poglej če je kakšno jabolko na poti do tarče
                obstacles = apples_on_path(get_distance(robot_pos, target), 100)
                if obstacles.__len__() > 0:
                    print("Na poti je drugo jabolko - menjam target")
                    current_apple = min(obstacles, key=get_distance_from_apple_to_robot)
                    target = get_apple_pos(current_apple)

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                # beleženje za izris grafa
                # file.write(str(target_angle) + ',' + str(time_now) + '\n')

                # Vmes bi radi tudi zavijali, zato uporabimo dva regulatorja.
                if state_changed:
                    # Ponastavi regulatorja PID.
                    get_straight_accel = 0.05
                    PID_frwd_base.reset()
                    PID_frwd_turn.reset()
                    timer_near_target = TIMER_NEAR_TARGET
                else:
                    if get_straight_accel < 1:
                        get_straight_accel += get_straight_accel_factor

                # Ali smo blizu cilja?
                robot_near_target = target_dist < DIST_NEAR
                if not robot_near_target_old and robot_near_target:
                    # Vstopili smo v bližino cilja.
                    # Začnimo odštevati varnostno budilko.
                    pid_frwd_base_multiplier = 0.5
                    timer_near_target = TIMER_NEAR_TARGET
                if robot_near_target:
                    timer_near_target = timer_near_target - loop_time
                robot_near_target_old = robot_near_target

                # Ali smo že na cilju?
                # Zadnjih nekaj obhodov zanke mora biti razdalja do cilja
                # manjša ali enaka DIST_EPS.
                err_eps = [d > DIST_EPS for d in robot_dist_hist]
                if sum(err_eps) == 0:
                    # Razdalja do cilja je znotraj tolerance, zamenjamo stanje.
                    print("Prišli smo na cilj")
                    claws_close()
                    if not encoder_apple_in_claws:
                        print("Nismo pobrali jabolko - enkoder")
                        claws_open()
                        state = State.GET_APPLE
                        continue

                    print("Pobrali smo jabolko - enkoder")
                    if get_apple_type(current_apple) == "appleBad":
                        print("Pobrali smo slabo jabolko")
                        state = State.ENEMY_HOME
                    else:
                        print("Pobrali smo dobro jabolko")
                        state = State.HOME

                elif timer_near_target < 0:
                    # Smo morda blizu cilja, in je varnostna budilka potekla?
                    speed_right = 0
                    speed_left = 0
                    state = State.GET_TURN

                else:
                    # multiplier v bližini cilja zmanjša PID, ker se tudi hitrost zmanjša
                    u_turn = PID_frwd_turn.update(
                        measurement=target_angle) * pid_frwd_base_multiplier * get_straight_accel
                    u_base = PID_frwd_base.update(measurement=target_dist) * get_straight_accel
                    # Omejimo nazivno hitrost, ki je enaka za obe kolesi,
                    # da imamo še manevrski prostor za zavijanje.
                    u_base = min(max(u_base, -SPEED_BASE_MAX), SPEED_BASE_MAX)
                    speed_right = (-u_base - u_turn)
                    speed_left = (-u_base + u_turn)

            elif state == State.HOME_TURN:
                # Obračanje robota na mestu, da bo obrnjen proti cilju.
                # print("State HOME_TURN")

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                # beleženje za izris grafa
                # file.write(str(target_angle) + ',' + str(time_now) + '\n')

                if state_changed:
                    # Če smo ravno prišli v to stanje, najprej ponastavimo PID.
                    PID_turn_apple.reset()

                if not encoder_apple_in_claws():
                    state = State.GET_APPLE
                    claws_open()
                    continue

                # Ali smo že dosegli ciljni kot?
                # Zadnjih nekaj obhodov zanke mora biti absolutna vrednost
                # napake kota manjša od DIR_EPS.
                err = [abs(a) > DIR_EPS for a in robot_dir_hist]

                if sum(err) == 0 or at_home(robot_pos):
                    # Vse vrednosti so znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    state = State.HOME_STRAIGHT
                else:
                    u = PID_turn_apple.update(measurement=target_angle)
                    speed_right = -u
                    speed_left = u

            elif state == State.HOME_STRAIGHT:
                # Vožnja robota naravnost proti ciljni točki.
                # print("State HOME_STRAIGHT")

                if at_home(robot_pos):
                    print("Smo že doma")
                    claws_open()
                    state = State.BACK_OFF
                    continue

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                # Vmes bi radi tudi zavijali, zato uporabimo dva regulatorja.
                if state_changed:
                    # Ponastavi regulatorja PID.
                    PID_frwd_base_apple.reset()
                    PID_frwd_turn_apple.reset()
                    timer_near_target = TIMER_NEAR_TARGET
                    pid_frwd_base_apple_multiplier = 1

                # Ali smo blizu cilja?
                robot_near_target = target_dist < DIST_NEAR
                if not robot_near_target_old and robot_near_target:
                    # Vstopili smo v bližino cilja.
                    # Začnimo odštevati varnostno budilko.
                    pid_frwd_base_apple_multiplier = 0.1
                    timer_near_target = TIMER_NEAR_TARGET
                if robot_near_target:
                    timer_near_target = timer_near_target - loop_time
                robot_near_target_old = robot_near_target

                # Ali smo že na cilju?
                # Zadnjih nekaj obhodov zanke mora biti razdalja do cilja
                # manjša ali enaka DIST_EPS.
                err_eps = [d > DIST_EPS for d in robot_dist_hist]
                if sum(err_eps) == 0 or at_home(robot_pos):
                    # Razdalja do cilja je znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    claws_open()
                    print("Prišli smo domov")
                    state = State.BACK_OFF

                elif timer_near_target < 0:
                    # Smo morda blizu cilja, in je varnostna budilka potekla?
                    speed_right = 0
                    speed_left = 0
                    state = State.HOME_TURN

                else:
                    # multiplier v bližini cilja zmanjša PID, ker se tudi hitrost zmanjša
                    u_turn = PID_frwd_turn_apple.update(measurement=target_angle) * pid_frwd_base_apple_multiplier
                    u_base = PID_frwd_base_apple.update(measurement=target_dist)
                    # Omejimo nazivno hitrost, ki je enaka za obe kolesi,
                    # da imamo še manevrski prostor za zavijanje.
                    u_base = min(max(u_base, -SPEED_BASE_MAX), SPEED_BASE_MAX)
                    speed_right = -u_base - u_turn
                    speed_left = -u_base + u_turn

            elif state == State.ENEMY_HOME_TURN:
                # Obračanje robota na mestu, da bo obrnjen proti cilju.
                # print("State ENEMY_HOME_TURN")

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                if state_changed:
                    # Če smo ravno prišli v to stanje, najprej ponastavimo PID.
                    PID_turn_apple.reset()

                if not encoder_apple_in_claws():
                    state = State.GET_APPLE
                    claws_open()
                    continue
                # Ali smo že dosegli ciljni kot?
                # Zadnjih nekaj obhodov zanke mora biti absolutna vrednost
                # napake kota manjša od DIR_EPS.
                err = [abs(a) > DIR_EPS for a in robot_dir_hist]

                if sum(err) == 0 or at_home_enemy(robot_pos):
                    # Vse vrednosti so znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    state = State.ENEMY_HOME_STRAIGHT
                else:
                    u = PID_turn_apple.update(measurement=target_angle)
                    speed_right = -u
                    speed_left = u

            elif state == State.ENEMY_HOME_STRAIGHT:
                # Vožnja robota naravnost proti ciljni točki.
                # print("State ENEMY_HOME_STRAIGHT")

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                # Vmes bi radi tudi zavijali, zato uporabimo dva regulatorja.
                if state_changed:
                    # Ponastavi regulatorja PID.
                    PID_frwd_base_apple.reset()
                    PID_frwd_turn_apple.reset()
                    timer_near_target = TIMER_NEAR_TARGET
                    pid_frwd_base_apple_multiplier = 1

                # Ali smo blizu cilja?
                robot_near_target = target_dist < DIST_NEAR
                if not robot_near_target_old and robot_near_target:
                    # Vstopili smo v bližino cilja.
                    # Začnimo odštevati varnostno budilko.
                    pid_frwd_base_apple_multiplier = 0.1
                    timer_near_target = TIMER_NEAR_TARGET
                if robot_near_target:
                    timer_near_target = timer_near_target - loop_time
                robot_near_target_old = robot_near_target

                # Ali smo že na cilju?
                # Zadnjih nekaj obhodov zanke mora biti razdalja do cilja
                # manjša ali enaka DIST_EPS.
                err_eps = [d > DIST_EPS for d in robot_dist_hist]
                if sum(err_eps) == 0 or at_home_enemy(robot_pos):
                    # Razdalja do cilja je znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    claws_open()
                    print("Prišli smo v nasprotnikov dom")
                    state = State.BACK_OFF

                elif timer_near_target < 0:
                    # Smo morda blizu cilja, in je varnostna budilka potekla?
                    speed_right = 0
                    speed_left = 0
                    state = State.ENEMY_HOME_TURN

                else:
                    # multiplier v bližini cilja zmanjša PID, ker se tudi hitrost zmanjša
                    u_turn = PID_frwd_turn_apple.update(measurement=target_angle) * pid_frwd_base_apple_multiplier
                    u_base = PID_frwd_base_apple.update(measurement=target_dist)
                    # Omejimo nazivno hitrost, ki je enaka za obe kolesi,
                    # da imamo še manevrski prostor za zavijanje.
                    u_base = min(max(u_base, -SPEED_BASE_MAX), SPEED_BASE_MAX)
                    speed_right = -u_base - u_turn
                    speed_left = -u_base + u_turn

            elif state == State.BACK_OFF:
                # print("State BACK_OFF")
                decelerate_both_motors_to(0, -500)
                if motor_grab.position < encoder_open:
                    claws_open()
                state = State.GET_APPLE

            elif state == State.CLEAR_HOME:
                bad_apples = bad_apples_at_home()
                if bad_apples.__len__() == 0:
                    state = State.GET_APPLE
                    continue

                target = bad_apples.pop()

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                speed_right = 0
                speed_left = 0

                # Preverimo, ali je robot na ciljni točki.
                # Če ni, ga tja pošljemo.
                if target_dist > DIST_EPS:
                    state = State.CLEAR_TURN
                    robot_near_target_old = False
                else:
                    state = State.CLEAR_OUT

            elif state == State.CLEAR_TURN:

                target_dist = get_distance(robot_pos, target)
                target_angle = get_angle(robot_pos, robot_dir, target)

                if state_changed:
                    # Če smo ravno prišli v to stanje, najprej ponastavimo PID.
                    PID_turn.reset()

                # Ali smo že dosegli ciljni kot?
                # Zadnjih nekaj obhodov zanke mora biti absolutna vrednost
                # napake kota manjša od DIR_EPS.
                err = [abs(a) > DIR_EPS for a in robot_dir_hist]

                if sum(err) == 0:
                    # Vse vrednosti so znotraj tolerance, zamenjamo stanje.
                    speed_right = 0
                    speed_left = 0
                    state = State.CLEAR_STRAIGHT

                else:
                    u = PID_turn.update(measurement=target_angle)
                    speed_right = -u
                    speed_left = u

            elif state == State.CLEAR_STRAIGHT:
                # Vožnja robota naravnost proti ciljni točki.

                # Predikcija kje se bomo nahajali v naslednji iteraciji
                # Če bi bili izven mape, gremo v stanje GET_TURN
                # Preverja 5 cm pred sabo
                if not is_point_on_map(point_transpose(robot_pos, robot_dir, 50)):
                    speed_left = 0
                    speed_right = 0
                    state = State.CLEAR_TURN
                    continue

                # Poglej če je target sploh še tam
                apple_pos = get_apple_pos(current_apple)
                if not (target.x - 50 < apple_pos.x < target.x + 50 and target.y - 50 < apple_pos.y < target.y + 50):
                    speed_left = 0
                    speed_right = 0
                    state = State.CLEAR_HOME
                    continue

                # Vmes bi radi tudi zavijali, zato uporabimo dva regulatorja.
                if state_changed:
                    # Ponastavi regulatorja PID.
                    get_straight_accel = 0.05
                    PID_frwd_base.reset()
                    PID_frwd_turn.reset()
                    timer_near_target = TIMER_NEAR_TARGET
                else:
                    if get_straight_accel < 1:
                        get_straight_accel += get_straight_accel_factor

                # Ali smo blizu cilja?
                robot_near_target = target_dist < DIST_NEAR
                if not robot_near_target_old and robot_near_target:
                    # Vstopili smo v bližino cilja.
                    # Začnimo odštevati varnostno budilko.
                    pid_frwd_base_multiplier = 0.5
                    timer_near_target = TIMER_NEAR_TARGET
                if robot_near_target:
                    timer_near_target = timer_near_target - loop_time
                robot_near_target_old = robot_near_target

                # Ali smo že na cilju?
                # Zadnjih nekaj obhodov zanke mora biti razdalja do cilja
                # manjša ali enaka DIST_EPS.
                err_eps = [d > DIST_EPS for d in robot_dist_hist]
                if sum(err_eps) == 0:
                    # Razdalja do cilja je znotraj tolerance, zamenjamo stanje.
                    print("Prišli smo na cilj")
                    claws_close()
                    if not encoder_apple_in_claws:
                        print("Nismo pobrali jabolko - enkoder")
                        claws_open()
                        state = State.CLEAR_HOME
                        continue

                    print("Pobrali smo jabolko - enkoder")
                    state = State.CLEAR_OUT

                elif timer_near_target < 0:
                    # Smo morda blizu cilja, in je varnostna budilka potekla?
                    speed_right = 0
                    speed_left = 0
                    state = State.CLEAR_TURN

                else:
                    # multiplier v bližini cilja zmanjša PID, ker se tudi hitrost zmanjša
                    u_turn = PID_frwd_turn.update(
                        measurement=target_angle) * pid_frwd_base_multiplier * get_straight_accel
                    u_base = PID_frwd_base.update(measurement=target_dist) * get_straight_accel
                    # Omejimo nazivno hitrost, ki je enaka za obe kolesi,
                    # da imamo še manevrski prostor za zavijanje.
                    u_base = min(max(u_base, -SPEED_BASE_MAX), SPEED_BASE_MAX)
                    speed_right = (-u_base - u_turn)
                    speed_left = (-u_base + u_turn)

            elif state == State.CLEAR_OUT:
                decelerate_both_motors_to(0, -500)
                claws_open()
                decelerate_both_motors_to(0, -300)
                state = State.CLEAR_HOME

            # Omejimo vrednosti za hitrosti na motorjih.
            speed_right = round(
                min(
                    max(speed_right, -SPEED_MAX),
                    SPEED_MAX)
            )
            speed_left = round(
                min(
                    max(speed_left, -SPEED_MAX),
                    SPEED_MAX)
            )

            # Vrtimo motorje.

            motor_right.run_forever(speed_sp=speed_right)
            motor_left.run_forever(speed_sp=speed_left)

            speed_right_old = speed_right
            speed_left_old = speed_left

        else:
            # Robot bodisi ni viden na kameri bodisi tema ne teče.
            motor_left.stop(stop_action='brake')
            motor_right.stop(stop_action='brake')

# Konec programa
robot_die()
