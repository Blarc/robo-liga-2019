#   tu je implementiran razred "PID"
from time import time


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
            time_now = time()
            delta_time = time_now - self._time
            self._time = time_now
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
