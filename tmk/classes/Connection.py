# tu je implementiran razred "Connection"

from io import BytesIO
import pycurl
import ujson
from time import time


class Connection:
    """
    Objekt za vzpostavljanje povezave s strežnikom.
    """

    def __init__(self, url: str):
        """
        Inicializacija nove povezave.

        Argumenti:
        url: pot do datoteke na strežniku (URL)
        """
        self._url = url
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
        except ValueError as err:
            if debug:
                print('Napaka pri razclenjevanju datoteke JSON: ' + str(err))
                print('Sporocilo streznika:')
                print(msg)
            return -1

    def test_delay(self, robot_die, num_iters: int = 10):
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
