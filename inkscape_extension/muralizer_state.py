import os
import math

from math import sqrt
import gettext
import os
import string
import sys
import time
import json


class MuralizerState:
	"""Encapsulates the state of the muralizer hardware

	This class is responsible for knowing all the parameters about
	the muralizer hardware and canvas, and for creating movement
	plans to given (x,y) coordinates.

	All linear distances internally are in millimeters, though the
	parameters are a mix of mm and cm (because asking people the
	size of their canvas in cm is a hard enough sell to
	Americans...).

	All radial distances are given in steps.

	The initial position is assumed to be that each spool is
	played out by canvasWidth.

	"""
	def __init__(self, *args, **kwargs):
		self.debug_path = "/tmp/muralizer_debug"
		self.debug_fd = file(self.debug_path, "w")

		self.alert("Set up state.")

		options = kwargs["options"]
		self.update_options(options)

		if "serialPort" in kwargs:
			self.serial_path = kwargs["serialPort"]

			self.serial_fd = serial.Serial(self.serial_path, 9600)
			self.has_serial = True
			header_line = self.serial_fd.readline()
			self.alert("Got header line: " + header_line.strip())

		else:
			# self.alert("Args: %s" % str(kwargs))

			self.serial_fd = file(os.devnull, "w")
			self.has_serial = False

			self.alert("Running without a real serial port.")
		
		

	def update_options(self, options):
		self.canvasWidth  = 10.0*options.canvasWidth  # cm to mm
		self.canvasHeight = 10.0*options.canvasHeight # 
		self.marginXL     = 10.0*options.marginXL     # 
		self.marginXR     = 10.0*options.marginXR     # 
		self.marginYT     = 10.0*options.marginYT     # 
		
		self.stepsPerRev  = 1.0*options.stepsPerRev    # count
		self.spoolDiameter = 1.0*options.spoolDiameter # mm
		
		self.stepMM = self.spoolDiameter*math.pi/self.stepsPerRev


		# XXX TODO There must be better bounds to use here
		self.MIN_R = (self.spoolDiameter+25) / self.stepMM
		self.MAX_R = math.sqrt(self.canvasHeight*self.canvasHeight + 
				       self.canvasWidth*self.canvasWidth) / self.stepMM


		# Figure out the initial coordinates...
 		initial_x = ( (self.canvasWidth-self.marginXR) + self.marginXL )/2
		initial_y = (self.canvasHeight + self.marginYT)/2

		self.r0 = self.calc_r0(initial_x, initial_y)
		self.r1 = self.calc_r1(initial_x, initial_y)

		self.initial_x = initial_x
		self.initial_y = initial_y

		#		self.r0 = round( self.canvasWidth/self.stepMM )   # steps
		#		self.r1 = round( self.canvasWidth/self.stepMM )   # steps


		# And splat it all out to our debug file
		to_dump = [
			(self.canvasWidth, "Canvas width, mm / canvasWidth"),
			(self.canvasHeight, "Canvas height, mm / canvasHeight"),
			(self.marginXL, "Margin left, mm / marginXL"),
			(self.marginXR, "Margin right, mm / marginXR"),
			(self.marginYT, "Margin top, mm / marginYT"),
			(self.stepsPerRev, "Steps per rev, steps / stepsPerRev"),
			(self.spoolDiameter, "Spool diameter, mm / spoolDiameter"),
			(self.stepMM, "Step length, mm / stepMM"),
			(self.r0, "r0 initial, steps / r0"),
			(self.r1, "r1 initial, steps / r1"),
			(self.MIN_R, "Min r, steps / MIN_R"),
			(self.MAX_R, "Max r, steps / MAX_R"),

			(initial_x, "Initial x / _computed_"),
			(initial_y, "Initial y / _computed_"),
			(self.abs_x(), "Current abs x / _computed_"),
			(self.abs_y(), "Current abs y / _computed_"),
			(self.area_x(), "Current drawing area x / _computed_"),
			(self.area_y(), "Current drawing area y / _computed_"), 

			]
		

		for x, s in to_dump:
			self.alert("Opt\t%.1f\t%s" % (x,s))
		
			
	def home(self):
		self.go_to_area(self.initial_x-self.marginXL, self.initial_y-self.marginYT)


	def attach_serial(self, serial_fd):
		if not self.has_serial:
			self.serial_fd.close()

		self.serial_fd = serial_fd
		self.has_serial = True

		self.alert("Attached serial.")

	def detach_serial(self):
		self.has_serial = False
		self.serial_fd = file("/dev/null", "w")

		self.alert("Detached serial.")

	def page_width(self):
		"""Get the width of the actual drawing area, in mm"""
		return (self.canvasWidth - self.marginXL - self.marginXR)


	def page_height(self):
		"""Get the height of the actual drawing area, in mm"""
		return (self.canvasHeight - self.marginYT)


	def __str__(self):
		retval = """Muralizer: Canvas: (%.1f,%.1f) to (%.1f,%.1f) (%.1f x %.1f cm, margins of %.1f-%.1f, %f).
Spools: %.1f mm diameter, %.1f steps/rev, %.3f mm resolution.
Cursor: <%.1f,%.1f>, which is  (%.2f,%.2f), (%.2f,%.2f) in print area.""" % ( 
			self.marginXL, self.marginYT,  self.canvasWidth-self.marginXR, self.canvasHeight,  self.canvasWidth, self.canvasHeight, self.marginXL, self.marginXR, self.marginYT,
			self.spoolDiameter, self.stepsPerRev, self.stepMM,
			self.r0, self.r1,
                        self.abs_x(), self.abs_y(),
                        self.area_x(), self.area_y())

		return retval

	def alert(self, str):
		self.debug_fd.write(str + "\n")
		self.debug_fd.flush()
		pass
		


	def clip_abs_xy(self, x, y):
		x_bounds = (self.marginXL, self.marginXL+self.page_width())
		y_bounds = (self.marginYT, self.marginYT+self.page_height())
		
		if x < x_bounds[0]:
			self.alert("x would violate left margin, clipping (%.1f < %.1f)" % (x,x_bounds[0]))
			x = x_bounds[0]
		elif x > x_bounds[1]:
			self.alert("x would violate right margin, clipping (%.1f > %.1f)" % (x, x_bounds[1]))
			x = x_bounds[1]

		if y < y_bounds[0]:
			self.alert("y would violate top margin, clipping (%.1f < %.1f)" % (y,y_bounds[0]))
			y = y_bounds[0]
		elif y > y_bounds[1]:
			self.alert("y would violate bottom margin, clipping (%.1f > %.1f)" % (y, y_bounds[1]))
			y = y_bounds[1]

		return (x,y)

	def abs_x(self):
		# Trivial derivation:
		#
		# Construct two right triangles across the top of the
		# canvas, with hypotenuses r0 and r1, both of height
		# y, with bases x and (width-x).
		#
		# Then:
		#
		#      x^2 + y^2 = r0^2 => y^2 = r0^2-x^2
		#  (w-x)^2 + y^2 = r1^2
		#
		#           w^2 - 2wx + x^2 + y^2 = r1^2
		#  w^2 - 2wx + x^2 + (r0^2 - x^2) = r1^2
		#                w^2 - 2wx + r0^2 = r1^2
		#  => w^2 + r0^2 - r1^2 = 2wx
		#  => x = (w^2 + r0^2 - r1^2)/2w
		#
		# Yay, my math degree was good for something!
		#
		
		return (self.r0*self.r0 - self.r1*self.r1 + self.canvasWidth*self.canvasWidth)/(2*self.canvasWidth)

	def abs_y(self):
		x = self.abs_x()
		return math.sqrt( self.r0*self.r0 - x*x )
		

	def area_x(self):
		return self.abs_x() - self.marginXL

	def area_y(self):
		return self.abs_y() - self.marginYT


	def calc_r0(self, x, y):
		(x,y) = self.clip_abs_xy(x,y)

		l = math.sqrt(x*x + y*y)
		r = round( l/self.stepMM )

		if r < self.MIN_R:
			self.alert("Move would violate r0 MIN_R, clipping")
			r = self.MIN_R

		if r > self.MAX_R:
 			self.alert("Move would violate r0 MAX_R, clipping")
			r = self.MAX_R

		return r

	def calc_r1(self, x, y):
		(x,y) = self.clip_abs_xy(x,y)

		xp = self.canvasWidth-x
		l = math.sqrt(xp*xp + y*y)

		r = round( l/self.stepMM )
		if r < self.MIN_R:
			self.alert("Move would violate r1 MIN_R, clipping")
			r = self.MIN_R

		if r > self.MAX_R:
			self.alert("Move would violate r1 MAX_R, clipping")
			r = self.MAX_R

		return r

	def go_to_area(self, x,y):
		xp = x + self.marginXL
		yp = y + self.marginYT
		
		r0p = self.calc_r0(xp,yp)
		r1p = self.calc_r1(xp,yp)


		command_summary = {
			"dest_area": (x,y),
			"dest_abs": (xp,yp),
			"cur_r": (self.r0,self.r1),
			"dest_r": (r0p, r1p),
			}
			

		self.alert("MOVE/ " + json.dumps(command_summary))
		self.cmd_move_rs(r0p, r1p)


	def _query(self, s):
		self.serial_fd.write(s + "\n")
		if self.has_serial:
			return self.serial_fd.readline().strip()
		else:
			return "-null serial-"


	####################
	# Command wrappers

	def ensure_setup(self, options):
		self.alert("CMD: ENSURE SETUP")

	def cmd_nop(self):
		self.alert("CMD: NOP")

	def cmd_disable_motors(self):
		self.alert("CMD: DISABLE MOTORS")

	def cmd_enable_motors(self):
		self.alert("CMD: ENABLE MOTORS")

	def cmd_move_rs(self, r0, r1):
		dr0 = r0 - self.r0
		dr1 = r1 - self.r1
		self.alert("CMD: WALK: Dest <%d, %d>, delta d<%d, %d>" % (r0, r1, dr0, dr1))
		self.r0 = r0
		self.r1 = r1

		q = "r %d %d" % (dr0, dr1)
		retval = self._query(q)
		if self.has_serial:
			time.sleep(max(abs(dr0),abs(dr1))*0.005)
		self.alert(" %s  => %s" % (q.strip(), retval))
		

	def cmd_move_r0(self, n):
		self.alert("CMD: WALK R0: %d" % n)
		self.cmd_move_rs(self.r0+n, self.r1)

	def cmd_move_r1(self, n):
		self.alert("CMD: WALK R1: %d" % n)
		self.cmd_move_rs(self.r0, self.r1+n)

	def cmd_version(self):
		self.alert("CMD: VERSION QUERY")

		if self.has_serial:
			return self._query("v")

		return "v0.1"

	def cmd_button_down(self):
		self.alert("CMD: BUTTON DOWN?")
		return False

	def cmd_pen_up(self):
		self.alert("CMD: RAISE PEN")
		return self._query("p u")

	def cmd_pen_down(self):
		self.alert("CMD: LOWER PEN")
		return self._query("p d")
	
	def cmd_pen_toggle(self):
		self.alert("CMD: TOGGLE PEN")

	def cmd_scram(self):
		self.alert("CMD: SCRAM!")

	def cmd_pause(self, dt):
		self.alert("CMD: PAUSE(%f)" % dt)
