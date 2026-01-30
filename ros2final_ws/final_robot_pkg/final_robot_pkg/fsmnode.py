#!/usr/bin/env python3
import time
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray

from .constants import (
    ROBOT_ACTIONS,
    TOPIC_LIVE_ACTION,
    TOPIC_START_ROBOT_MOVEMENT,
)


class FSMNode(Node):

    def __init__(self):
        super().__init__("fsm_node")

        # -------------------------
        # FSM States
        # -------------------------
        self.WAITING  = "WAITING"
        self.LOADING  = "LOADING"
        self.CHECKING = "CHECKING"
        self.MOVING   = "MOVING"
        self.CRASH    = "CRASH"
        self.GOBACK   = "GOBACK"

        self.state = self.WAITING
        self.last_state = None

        self.get_logger().info("FSM started.")

        # -------------------------
        # Cargo
        # -------------------------
        self.loaded_tags = set()
        self.CARGO_MAX_LOAD = 8

        # -------------------------
        # Publishers
        # -------------------------
        self.move_pub = self.create_publisher(String, TOPIC_START_ROBOT_MOVEMENT, 10)
        self.tts_pub  = self.create_publisher(String, "tts_text", 10)
        self.led_pub  = self.create_publisher(Int32MultiArray, "led_color", 10)

        # -------------------------
        # Subscribers
        # -------------------------
        self.create_subscription(String, TOPIC_LIVE_ACTION, self.move_status_cb, 10)
        self.create_subscription(String, "rfid_tag", self.rfid_cb, 10)
        self.create_subscription(String, "button_event", self.button_cb, 10)

        # -------------------------
        # internal flags
        # -------------------------
        self.command_sent = False
        self.tts_sent = False
        self.led_sent = False
        self.movement_done = False

        # Timer
        self.timer = self.create_timer(0.05, self.fsm_loop)


    # ---------------------------------------------------
    # Helpers (with ONCE protection)
    # ---------------------------------------------------

    def send_led(self, r, g, b):
        if self.led_sent:
            return
        msg = Int32MultiArray()
        msg.data = [r, g, b]
        self.led_pub.publish(msg)
        self.led_sent = True
        self.get_logger().info(f"[LED] -> {msg.data}")

    def say(self, text):
        if self.tts_sent:
            return
        self.tts_pub.publish(String(data=text))
        self.tts_sent = True
        self.get_logger().info(f"[TTS] {text}")

    def send_move_cmd(self, action):
        if self.command_sent:
            return
        self.move_pub.publish(String(data=action))
        self.command_sent = True
        self.get_logger().info(f"[MOVE] {action}")

    def transition(self, new_state):
        self.get_logger().info(f"STATE {self.state} -> {new_state}")
        self.last_state = self.state
        self.state = new_state

        # reset ONCE flags
        self.command_sent = False
        self.tts_sent = False
        self.led_sent = False

        # SUPER IMPORTANT
        self.movement_done = False



    # ---------------------------------------------------
    # Callbacks
    # ---------------------------------------------------

    def rfid_cb(self, msg):
        if self.state == self.LOADING:
            self.loaded_tags.add(msg.data)
            self.get_logger().info(f"[RFID] Loaded {msg.data}")

    def button_cb(self, msg):
        btn = msg.data.strip()

        if btn == "LOAD":
            if self.state == self.WAITING:
                self.transition(self.LOADING)
            elif self.state == self.LOADING:
                self.transition(self.WAITING)

        elif btn == "START":
            if self.state == self.WAITING:
                self.transition(self.CHECKING)

        elif btn == "RESET":
            if self.state == self.WAITING:
                self.transition(self.GOBACK)

    def move_status_cb(self, msg):
        if msg.data.endswith("END"):
            self.movement_done = True
            self.get_logger().info("[MOVE END]")


    # ---------------------------------------------------
    # Logic
    # ---------------------------------------------------

    def is_overloaded(self):
        cargo = len(self.loaded_tags)
        self.get_logger().info(f"[CARGO] {cargo}")
        if cargo <= self.CARGO_MAX_LOAD:
            return False
        else: # exceeding probabilities, tarting at 0.6, adding 0.05 per additional cargo piece
            probability = 0.60 + (0.05 * (cargo - self.CARGO_MAX_LOAD))

        result = (random.random() < probability)
        self.get_logger().info(f"[CARGO_MAX_LOAD] {probability}, {result}")
        return result


    # ---------------------------------------------------
    # FSM MAIN LOOP
    # ---------------------------------------------------

    def fsm_loop(self):

        # ---------------- WAITING ----------------
        if self.state == self.WAITING:
            self.send_led(255, 255, 0)  # yellow
            self.say("Waiting.")
            return

        # ---------------- LOADING ----------------
        if self.state == self.LOADING:
            self.send_led(0, 255, 0)  # green
            self.say("Loading cargo.")
            return

        # ---------------- CHECKING ----------------
        if self.state == self.CHECKING:
            self.send_led(255, 255, 0)
            self.say("Checking cargo weight.")

            if self.is_overloaded():
                self.transition(self.CRASH)
            else:
                self.transition(self.MOVING)
            return

        # ---------------- MOVING ----------------
        if self.state == self.MOVING:
            self.send_led(0, 0, 255)
            self.say("Sailing to the other side.")

            self.send_move_cmd(ROBOT_ACTIONS["LEFT_TO_RIGHT"])

            if self.movement_done:
                self.say("Arrived at destination.")
                self.transition(self.WAITING)
            return

        # ---------------- CRASH ----------------
        if self.state == self.CRASH:
            self.send_led(255, 0, 0)
            self.say("The ship has crashed!")

            self.send_move_cmd(ROBOT_ACTIONS["LEFT_TO_MIDDLE_TO_LEFT"])

            if self.movement_done:
                self.loaded_tags = set()
                self.transition(self.WAITING)
            return

        # ---------------- GOBACK ----------------
        if self.state == self.GOBACK:
            self.send_led(0, 0, 255)
            self.say("Returning home.")

            self.send_move_cmd(ROBOT_ACTIONS["RIGHT_TO_LEFT"])

            if self.movement_done:
                self.loaded_tags = set()
                self.transition(self.WAITING)
            return



def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

