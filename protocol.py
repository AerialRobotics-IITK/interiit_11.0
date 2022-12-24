import serial


class Protocol:
    """
    This class defines the protocol and deals with the communication with the flight controller.
    """

    def __init__(self, IP, PORT, baudrate=115200):
        """
        Arguments : IP is the IP address of the flight controller
                    PORT is the port number of the flight controller
        Initializes the protocol object
        """
        self.com: serial = serial.serial_for_url(
            "socket://" + IP + ":" + str(PORT), timeout=1
        )
        self.com.baudrate: int = baudrate

        self.raw_commands: list[int] = [1500, 1500, 900, 1500, 1500, 1500, 1500, 2000]

        # type of payload
        self.SET_RAW_RC: int = 200

        # payload length
        self.SET_RAW_RC_LENGTH: int = 16

        # Limits for the arm field in SET_RAW_RC
        self.ARM_UPPER: int = 1700
        self.ARM_LOWER: int = 1300

        # payload byte lengths
        self.SET_RAW_RC_BYTE_LENGTHS: list[int] = [2, 2, 2, 2, 2, 2, 2, 2]
        # Length of the response given by the server to an IN packet
        self.COMMAND_RESP_LENGTH: int = 6

        # Header and direction for the packets
        self.HEADER: str = "24 4d"
        self.DIRECTION: str = "3c"

        # Composition of packet
        self.HEADER_BYTES: int = 2
        self.DIRECTION_BYTES: int = 1
        self.MSG_LENGTH_BYTES: int = 1
        self.TYPE_OF_PAYLOAD_BYTES: int = 1
        self.CHECKSUM_BYTES: int = 1

    def make_message(
        self,
        msg_length: int = 0,
        type_of_payload: int = -1,
        payload: list = [],
        byte_lengths: list = [],
    ) -> bytes:

        """
        Constructs a message to be sent to the server from the payload
        Arguments : msg_length is the length of the payload in bytes. To be ket 0 in the case of an OUT packet
                    type_of_payload is the type of payload.
                    payload is a list of integers to be sent to the server
                    byte_lengths is a list of the number of bytes to be used to represent each element of the payload

        Returns : message is the message to be sent to the server
        """

        header: str = self.HEADER  # Header remains constant for all messages
        direction: tuple[
            str
        ] = (
            self.DIRECTION
        )  # Direction is always 3c for any packet sent by the flight controller

        # Convert the integers to bytes, NOTE: the byteorder is little endian for the payload
        msg_length: bytes = msg_length.to_bytes(self.MSG_LENGTH_BYTES, byteorder="big")
        type_of_payload: bytes = type_of_payload.to_bytes(
            self.TYPE_OF_PAYLOAD_BYTES, byteorder="big"
        )
        pl: bytes = b""
        for i in range(0, len(payload)):
            pl = pl + payload[i].to_bytes(byte_lengths[i], byteorder="little")

        # checksum is the XOR of all the bytes in the payload and the type of payload and the length of the payload
        checksum = msg_length[0] ^ type_of_payload[0]
        for i in range(0, len(pl)):
            checksum = checksum ^ pl[i]
        checksum = checksum.to_bytes(self.CHECKSUM_BYTES, byteorder="big")
        message: bytes = (
            bytes.fromhex(header)
            + bytes.fromhex(direction)
            + msg_length
            + type_of_payload
            + pl
            + checksum
        )
        return message

    def send(self, msg) -> None:
        """
        Sends a message to the server
        Arguments : msg is the message to be sent to the server
        Returns : None
        """
        self.com.write(msg)

    def close(self) -> None:
        """
        Closes the connection to the server
        Arguments : None
        Returns : None
        """

        self.com.close()

    def read(self, size:int) -> int:
        """
        Reads bytes from the server
        Arguments : size is the number of bytes to be read
        Returns : the bytes read from the server
        """

        return self.com.read(size)

    def is_armed(self) -> True|None:
        """
        Checks if the drone is armed
        Arguments : None
        Returns : True if the drone is armed, False otherwise
        """

        if (
            self.raw_commands[-1] > self.ARM_LOWER
            and self.raw_commands[-1] < self.ARM_UPPER
        ):
            return True

    def arm(self) -> None:
        """
        Arms the drone
        Arguments : None
        Returns : None
        """

        self.raw_commands[-1] = 1500
        msg:bytes = self.make_message(
            msg_length=self.SET_RAW_RC_LENGTH,
            type_of_payload=self.SET_RAW_RC,
            payload=self.raw_commands,
            byte_lengths=self.SET_RAW_RC_BYTE_LENGTHS,
        )
        self.send(msg)
        self.read(self.COMMAND_RESP_LENGTH)

    def disarm(self) -> None:
        """
        Disarms the drone
        Arguments : None
        Returns : None
        """

        self.raw_commands[-1] = self.ARM_LOWER - 1
        msg:bytes = self.make_message(
            msg_length=self.SET_RAW_RC_LENGTH,
            type_of_payload=self.SET_RAW_RC,
            payload=self.raw_commands,
            byte_lengths=self.SET_RAW_RC_BYTE_LENGTHS,
        )
        self.send(msg)
        self.read(self.COMMAND_RESP_LENGTH)

    def set_RPY_THR(self, roll=None, pitch=None, yaw=None, throttle=None) -> None:
        """
        Sets the roll, pitch, yaw and throttle values of the drone
        Will set only those values for which the argument is not None
        Others will remain unchanged
        Arguments : roll, pitch, yaw and throttle are the values to be set
        Returns : None
        """

        if not self.is_armed():
            # warn if the dronse is not yet armed
            print("WARNING : Drone not armed")
        if roll is not None:
            self.raw_commands[0] = roll
        if pitch is not None:
            self.raw_commands[1] = pitch
        if yaw is not None:
            self.raw_commands[2] = throttle
        if throttle is not None:
            self.raw_commands[3] = yaw

        msg:bytes = self.make_message(
            msg_length=self.SET_RAW_RC_LENGTH,
            type_of_payload=self.SET_RAW_RC,
            payload=self.raw_commands,
            byte_lengths=self.SET_RAW_RC_BYTE_LENGTHS,
        )
        self.send(msg)
        self.read(self.COMMAND_RESP_LENGTH)
