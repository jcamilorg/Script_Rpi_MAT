"""
@author: M.A.T
"""
# importamos "time" para controlar la frecuencia con la que se recopilan los
# datos del sensor e importamos "w1thermsensor" y "Adafruit" para permitir que
# nuestro proyecto se comunique con el módulo ADC y el sensor de temperatura.
# Adem[as de eso, importamos "paho.mqtt" para realizar la transmision por MQTT
# Importamos RPI.GPIO para permitir que nuestro código se comunique con el sensor de ultrasonido y actuadors ___________________________________
import RPi.GPIO as GPIO
import ssl
import time
from w1thermsensor import W1ThermSensor
import Adafruit_ADS1x15
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
from multiprocessing import Process
import socket
import smtplib
import os
import sys

# Datos para poder enviar el mensaje por Email
sender_address = "scorsomeone@gmail.com"
sender_password = "sHMX7m12"
sender_server = 'smtp.gmail.com'
sender_port = 587
recipient_address = "juancamilorg64@gmail.com"

# Conexion a ThingSpeak, definimos el ID, la clave y el Host: _____________________________________________________________________________
channelID = "1353138"  # El ID del canal de thingSpeak
apiKey = "APLXQ2T9S3DYXAED"  # Poner el WriteAPI key
mqttHost = "mqtt.thingspeak.com"  # host de ThingSpeak

# Permitir la conexion SSL Websockets
tTransport = "websockets"
tTLS = {'ca_certs': "/etc/ssl/certs/ca-certificates.crt",
        'tls_version': ssl.PROTOCOL_TLSv1}
tPort = 443

# Subscripcion al topic de ThingSpeak
topic = "channels/" + channelID + "/publish/" + apiKey

# Crear objetos y definir variables para utilizar los sensores_________________________________________________________________________________

# Se crea un objeto para almacenar una conexión con el sensor de temperatura.
sensor = W1ThermSensor()

# Se crea un objeto para almacenar la conexión con nuestro módulo ADC
adc = Adafruit_ADS1x15.ADS1115()

# Se escoge un valor de ganancia para elegir el valor máximo del sensor de turbidez
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# En este caso se elige 1
GAIN = 1

# GPIO Mode (BOARD / BCM) y se desactivan las advertencias
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)  # no muestra las advertencias

# Configurar los pines del los actuadores
Cal = 13  # pin calentador
M1 = 5  # pin motor 1 - Cambio de Agua
M2 = 6  # motor 2   - Alimentación
# Se configuran los pines del ultrasonido
GPIO_TRIGGER = 23
GPIO_ECHO = 24

# Se definen los pines de los actuadores como pines de salida
GPIO.setup(Cal, GPIO.OUT)
GPIO.setup(M1, GPIO.OUT)
GPIO.setup(M2, GPIO.OUT)
# Se configura el tipo de pin del TRIGGER y del ECHO (entrada o salida)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)


# Creacion de funciones para simplificar codigo _____________________________________________________________________________
def ejecutarConTiempoLimite(func, time):
    """
    Ejecuta una función con un limite de tiempo
    Tiene que recibir:
        func: el nombre de la función a ejecutar
        args: una tupla con los argumentos a pasar a la función
    Devuelve True si ha finalizado la función correctamente

    https://docs.python.org/2/library/multiprocessing.html
    """

    p = Process(target=func)
    p.start()
    p.join(time)
    if p.is_alive():
        p.terminate()
        print("Ha finalizado por timeout")
        return False

    return True


def enviar_email(text):
    try:
        message = "From: " + sender_address + "\nTo: " + \
            recipient_address + "\nSubject: M.A.T-IoT\n\n" + text

        server = smtplib.SMTP(sender_server, sender_port)
        server.ehlo()
        server.starttls()
        server.login(sender_address, sender_password)
        server.sendmail(sender_address, recipient_address, message)
        server.close()
    except:
        print("No se pudo enviar el email")


def distance():
    # Reiniciar pines
    GPIO.output(GPIO_TRIGGER, False)
    GPIO.output(GPIO_TRIGGER, False)

    # Se pone el TRIGGER en alto
    GPIO.output(GPIO_TRIGGER, True)

    # Se pone el TRIGGER en bajo después de 0.01ms
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # Se guarda el tiempo de inicio
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # Se guarda el tiempo de llegada de la señal
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # Se calcula el tiempo de diferencia de la salida y llegada de la señal
    TimeElapsed = StopTime - StartTime
    # Se multiplica por la velocidad del sonido y se divide entre dos debido a
    # que la señal va de ida y vuelta
    distance = (TimeElapsed * 34300) / 2

    return distance


def Leer_sensores(stempi, sturbi, sdisti):
    global tempi, turbi, disti
    if stempi == "tempi":
        # Se obtiene el valor de temperatura y se almacena en tempi
        try:
            tempi = sensor.get_temperature()
        except:
            pass

    if sturbi == "turbi":
        # Se toma el valor de salida del sensor de ultrasonido y se almacena en la variable dist
        disti = distance()

    if sdisti == "disti":
        # Se lee el canal 0 del ADC
        turbi = adc.read_adc(0, gain=GAIN)
        turbi = 100 - turbi*100/(2**15)


def Encender(sensor):
    GPIO.output(sensor, True)


def Apagar(sensor):
    GPIO.output(sensor, False)


def Enviar_thingspeak(tempi, disti, turbi):
    # Se crea el Mensaje (la carga util) para enviar al servidor de thingspeak
    tPayload = "field1=" + str(tempi) + " temperatura" + "&field2=" + str(turbi) + \
        " turbidez" + "&field3=" + \
        str(disti) + " distancia" + "&field4=" + time.ctime()
    # Se envian las variables a ThingSpeak por MQTT
    publish.single(topic, payload=tPayload, hostname=mqttHost,
                   port=tPort, tls=tTLS, transport=tTransport)

    # imprimir los datos en pantalla
    print("")
    # Se imprime el valor de temperatura
    print("La temperatura es %s celsius" % tempi)
    # Se imprime el valor de distancia en la consola
    print("Distancia medida = %.1f cm" % disti)
    # Se imprime el valor de turbidez
    print("turbidez = %.1f " % turbi)
    print(time.ctime())


def on_connect(client, userdata, flags, rc):
    cliente.subscribe("mat-iot/#")
    print("Connected with result code "+str(rc))


def on_message(client, userdata, msg):
    # importar variables locales para poder utilizarlas
    global Auto
    global inter
    global Cal_Est
    # Imprimir el mensaje recibido
    print(msg.topic+" "+str(msg.payload))

    if(msg.topic == 'mat-iot/') and (str(msg.payload) == "b'Auto'"):
        Auto = True
        cliente.publish('mat-iot/modo', payload="Se activó el modo áutomatico")
        cliente.publish('mat-iot/modo/auto', payload="AutoOk")

    elif(msg.topic == 'mat-iot/') and (str(msg.payload) == "b'noAuto'"):
        Auto = False
        cliente.publish(
            'mat-iot/modo', payload="Se desactivo el modo áutomatico")
        cliente.publish('mat-iot/modo/noauto', payload="Ok")

    if not(Auto):
        # Logica de control para manejo manual de los actuadores.
        # Logica calentador
        if (msg.topic == 'mat-iot/tempi') and (str(msg.payload) == "b'True'"):
            Encender(Cal)
            Cal_Est = True
        elif (msg.topic == 'mat-iot/tempi') and str(msg.payload) == "b'False'":
            Apagar(Cal)
            Cal_Est = False

        # logica cambio de agua
        if (msg.topic == 'mat-iot/M1') and (str(msg.payload) == "b'True'"):
            Encender(M1)

        elif (msg.topic == 'mat-iot/M1') and str(msg.payload) == "b'False'":
            Apagar(M1)

        # logica Alimentación
        if (msg.topic == 'mat-iot/M2') and (str(msg.payload) == "b'True'"):
            Encender(M2)

        elif (msg.topic == 'mat-iot/M2') and str(msg.payload) == "b'False'":
            Apagar(M2)

        # logica Todos
        if (msg.topic == 'mat-iot/all') and (str(msg.payload) == "b'True'"):
            Encender(Cal)
            Encender(M1)
            Encender(M2)

        # logica intermitente
        if (msg.topic == 'mat-iot/tempi') and (str(msg.payload) == "b'inter'"):
            inter = True
            print("inter")

    if (msg.topic == 'mat-iot/all') and str(msg.payload) == "b'False'":
        Apagar(Cal)
        Apagar(M1)
        Apagar(M2)
        inter = False
        Auto = False
        cliente.publish('mat-iot/modo/noauto', payload="Ok")


# Definicion de variables globales para diferentes paramentros del control___________________________________________________________________________________________

# Variables globales para valor de los sensores
tempi = 0  # temperatura
turbi = 0  # turbidez
disti = 0  # distancia

# Variable que indica que modo se esta utilizando| si se esta trabajando en automatico o en manual.
Auto = True


nprint = 15  # Se defini para evitar realizar el envio de los datos en cada iteración

inter = False  # Varible que define si se esta en modo intermitente para el calentador

Cal_Est = False  # Variable que define en que estado esta el calentador

# Crear el cliente MQTT
cliente = mqtt.Client()

cliente.on_connect = on_connect  # Asignar la función al envento de conectarse
cliente.on_message = on_message  # Asignar la función al evento de recibir mensaje
cliente.publish('mat-iot/modo/auto', payload="AutoOk")

# Conentarse al servidor publico para realizar la comunicación
cliente.connect("test.mosquitto.org", 1883, 10)

# Variable que registra la ultima vez que se envio un email
tiempo_envio_email = "dia mes dia# 00:00:00 año"

# Ejecución # Se utiliza while True loop para ejecutar el código dentro de él para siempre. _____________________________________________________________________________________________________________________
while True:
    # Lectura de los sensores
    ejecutarConTiempoLimite(Leer_sensores("tempi", "turbi", "disti"), 1)
    # Obtener la hora actual.
    tiempo = time.ctime()
    # Enviar los datos segun la varible nprint para evitar enviar en cada iteración.
    if nprint == 15:  # enviar datos cada 15 segundos
        # leer los sensores con un tiempo maximo de un segundo
        Enviar_thingspeak(tempi, disti, turbi)
        nprint = 0
    else:
        nprint = nprint+1

    # Manejo de los actuadores via pagina web y aplicación movil_______________________________________________________________________________
    cliente.loop()  # Revisar si existe algun mensaje de MQTT - en dado caso realizar la función
    # Logica que permite tener el modo intermitente en el calentador.____________________________________________________________________
    if inter and Cal_Est:
        GPIO.output(Cal, False)
        Cal_Est = False
    elif inter and not Cal_Est:
        GPIO.output(Cal, True)
        Cal_Est = True

    # Modo automatico sistema_________________________________________________________________________________________________________________
    if Auto:
        # Logica para el manejoTemperatura:
        umbral_temp = 28
        if tempi <= umbral_temp:  # 28 grados es el valor ideal para criar tilapias
            Encender(Cal)
            vtiempo = tiempo.split()
            vtiempoe = tiempo_envio_email.split()
            vhora = vtiempo[3].split(":")
            vhorae = vtiempoe[3].split(":")
            # compara si se encuentra en el mismo minuto- si no envia el mensaje para no satura el email
            if vhora[1] != vhorae[1]:
                print("enviando email")
                mensaje = "M.A.T ioT info \n\nEstimado usuario, le iformamos que la planta de tilapia no esta operando en las mejores condiciones ya que la temperatura es muy baja menor a " + \
                    str(umbral_temp) + \
                    "\n\nLa temperatura medida fue: " + str(tempi)
                enviar_email(mensaje)
                tiempo_envio_email = tiempo
                print("se envio el mensaje")
        else:
            Apagar(Cal)

        # logica para control del cambio de Agua:
        umbral_turbi = 80
        if turbi >= umbral_turbi:  # Cuando exista un alto nivel de turbidez cambiar agua
            Encender(M1)
            vtiempo = tiempo.split()
            vtiempoe = tiempo_envio_email.split()
            vhora = vtiempo[3].split(":")
            vhorae = vtiempoe[3].split(":")
            # compara si se encuentra en el mismo minuto- si no envia el mensaje para no satura el email
            if vhora[1] != vhorae[1]:
                mensaje = "M.A.T ioT info \n\nEstimado usuario, le iformamos que la planta de tilapia no esta operando en las mejores condiciones optimas ya que se detecto una alta turbidez" + \
                    str(umbral_turbi) + \
                    "\n\nLa Turbidez medida fue: " + str(turbi)
                enviar_email(mensaje)
                tiempo_envio_email = tiempo

        else:
            Apagar(M1)

        # Logica para el manejo de la alimentación:
        # Se requiere una alimentación de 3 veces al dia a las 10:00Am - a las 1:00pm y a las 8:00pm
        vtime = tiempo.split()
        hora = vtime[3]
        # Encender el motor para alimentar los peces
        if (hora == "10:00:00" or hora == "13:00:00" or hora == "20:00:00"):
            Encender(M2)
            print("Se alimentarón los peces a las: " + hora)
        # Apagar el motor de alimentar a todos los peces 15 segundos despues
        elif (hora == "10:00:15" or hora == "13:00:15" or "20:00:15"):
            Apagar(M2)

    # Se espera 15 segundos para tomar la siguiente muestra
    time.sleep(0.5)
