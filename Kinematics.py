import time
from typing import List, Tuple, Union
from math import pi
import numpy as np
import numpy.matlib as npm

import unittest
import roboticstoolbox as rtb
import matplotlib.pyplot as plt


class Robot:
	MAX_VELOCITY = 15
	MAX_ACCELERATION = 50
	DT = 0.033

	L2l1 = 0.096
	L2l2 = 0.516
	L2l3 = 0.04009
	L2h1 = 0.14454
	L3 = 0.04009
	LRCC = 0.4318
	Ltool = 0.416
	Lpitch2yaw = 0.0091
	Lyaw2ctrlpnt = 0.106

	# Create the manipulator
	serialManipulator = rtb.DHRobot(
		[
			rtb.RevoluteMDH(a=0, d=0, alpha=pi/2,offset=pi/2, qlim=[-pi/2,pi/2]), 		                      # q1
			rtb.RevoluteMDH(a=0, d=0, alpha=-pi/2, offset=-pi/2, qlim=[-pi/2,pi/2]), 	                      # q(2)
            rtb.PrismaticMDH(a=0, theta=0, alpha=pi/2, offset=-0.4318, qlim=[3*0.0254,15.0*0.0254]),	# q(3)
            rtb.RevoluteMDH(a=0, d=0.416, alpha=0, offset=0, qlim=[-pi/2,pi/2]),				              # q(4)
            rtb.RevoluteMDH(a=0, d=0, alpha=-pi/2, offset=-pi/2, qlim=[-pi/2,pi/2]),		              # q(5)
            rtb.RevoluteMDH(a=0.0091, d=0, alpha=-pi/2, offset=-pi/2, qlim=[-pi/3,pi/3]),             # q(6)
            rtb.RevoluteMDH(a=0, d=0.0102, alpha=-pi/2, offset=0, qlim=[0, 0])
		], name="simplified dVRK")

	def __init__(self) -> None:
		self.q = np.zeros(7)
		
		# Display Robot
		self.serialManipulator.teach(self.q)
		
		# Calculate the screw axes
		self.S = np.array([[0,-1,0,0,0,0],  	                                # (1)
			[-1,0,0,0,0,0],						                                          # (2)
			[-1,0,0,0,0,-self.L2l3],					                                  # (2')
			[-1,0,0,0,-self.L2h1,-self.L2l3],				                            # (2'')
     		[-1,0,0,0,-self.L2h1,(self.L2l2 - self.L2l3)], 	                  # (2'''')
     		[0,0,0,0,0,-1],						                                        # (3)
     		[0,0,-1,-(self.L2l2 - self.L2l3 + self.L3),0,0],	                #(4)
     		[-1,0,0,0,-(self.LRCC-self.Ltool),(self.L2l2-self.L2l3+self.L3)], # (5)
     		[0,-1,0,(self.LRCC-self.Ltool-self.Lpitch2yaw),0,0],	            # (6)
     		[0,-1,0,(self.LRCC-self.Ltool-self.Lpitch2yaw),0,0]], dtype=object).T

		# Robot Home Configuration
		self.M = np.array([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, -1, (self.LRCC-self.Ltool-self.Lpitch2yaw-self.Lyaw2ctrlpnt)], [0, 0, 0, 1]], dtype=object)

	@classmethod
	def forward(cls, q: Tuple[float, float, float]): 
		T = self.serialManipulator.fkine(self.S, self.M, q)

	@classmethod
	def inverse(cls, T: Tuple[float, float, float]): 
		print(T)
		sol = cls.serialManipulator.ikine_LM(T)
		# cls.arm.plot(sol.q)

	@classmethod
	def check_angle_limits(cls, theta: float) -> bool:
		return
	@classmethod
	def max_velocity(cls, all_theta: List[float]) -> float:
		return

	@classmethod
	def max_acceleration(cls, all_theta: List[float]) -> float:
		return

	@classmethod
	def min_reachable_radius(cls) -> float:
		return

	@classmethod
	def max_reachable_radius(cls) -> float:
		return


def main() -> None:
	robot = Robot()

if __name__ == '__main__':
	main()
