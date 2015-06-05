This project is not completed yet.
Here are some design ideas:

Senor is attached in the middle of the quardcopter.
The position of the sensor is toward the motor 0(M0).
When the sum of the PMW values of four motors is larger than 68*4, the quadcopter is able to take off.
The HW reference is from the website: http://csenichijou.blogspot.tw/2014/03/Quadcopter-1.html#more


	M0  (Pitch +)
	 |
	 |
M2 ------------- M1
(Roll+)	 |	(Roll-)
	 |
	M3  (Pitch -)


	M0  (accel_x -)
	 |
	 |
M2 ------------- M1
(y +)	 |	(y -)
	 |
	M3  (accel_x +)
