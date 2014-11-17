# muralizer.py
#
# Version 0.0.1 2013-09-28
#
# This is built very heavily on 
# 
#    # eggbot.py
#    # Part of the Eggbot driver for Inkscape
#    # http://code.google.com/p/eggbotcode/
#    #
#    # Version 2.3.4, dated 8/11/2013
#    #
#
# muralizer.py is released under the same terms as eggbot.py:
# 
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

# TODO: Add and honor advisory locking around device open/close for non Win32

# TODO: Reintroduce serial port scans
# TODO: Complete refactor of #MIP functions, replace them with #MCLs.
# TODO: Remove all the ugly/diff-stracting #MCL annotations once they're done.
# TODO: Reintroduce eggbot-specific code and make this a generic drawbot interface.

# Python imports
from math import sqrt
import gettext
import os
import string
import sys
import time
import json

# PySerial import
import serial

# Inkscape imports
from bezmisc import *
from simpletransform import *
import simplepath
import cspsubdiv

# Drawbot imports
#import eggbot_scan
from muralizer_state import MuralizerState

F_DEFAULT_SPEED = 1
N_PEN_DOWN_DELAY = 400    # delay (ms) for the pen to go down before the next move
N_PEN_UP_DELAY = 400      # delay (ms) for the pen to up down before the next move
N_PAGE_HEIGHT = 800       # Default page height (each unit equiv. to one step)
N_PAGE_WIDTH = 3200       # Default page width (each unit equiv. to one step)

N_PEN_UP_POS = 50      # Default pen-up position
N_PEN_DOWN_POS = 40      # Default pen-down position
N_SERVOSPEED = 50			# Default pen-lift speed
N_WALK_DEFAULT = 10		# Default steps for walking stepper motors
N_DEFAULT_LAYER = 1			# Default inkscape layer

# if bDebug = True, create an HPGL file to show what is being plotted.
# Pen up moves are shown in a different color if bDrawPenUpLines = True.
# Try viewing the .hpgl file in a shareware program or create a simple viewer.

bDebug = False
miscDebug = False
bDrawPenUpLines = False
bDryRun = False # write the commands to a text file instead of the serial port

platform = sys.platform.lower()

HOME = os.getenv( 'HOME' )
if platform == 'win32':
	HOME = os.path.realpath( "C:/" )  # Arguably, this should be %APPDATA% or %TEMP%

DEBUG_OUTPUT_FILE = os.path.join( HOME, 'test.hpgl' )
DRY_RUN_OUTPUT_FILE = os.path.join( HOME, 'dry_run.txt' )
MISC_OUTPUT_FILE = os.path.join( HOME, 'misc.txt' )


def parseLengthWithUnits( str ):  # MCL
	'''
	Parse an SVG value which may or may not have units attached
	This version is greatly simplified in that it only allows: no units,
	units of px, and units of %.  Everything else, it returns None for.
	There is a more general routine to consider in scour.py if more
	generality is ever needed.
	'''
	u = 'px'
	s = str.strip()
	if s[-2:] == 'px':
		s = s[:-2]
	elif s[-1:] == '%':
		u = '%'
		s = s[:-1]

	try:
		v = float( s )
	except:
		return None, None

	return v, u

def subdivideCubicPath( sp, flat, i=1 ): # MCL
	"""
	Break up a bezier curve into smaller curves, each of which
	is approximately a straight line within a given tolerance
	(the "smoothness" defined by [flat]).

	This is a modified version of cspsubdiv.cspsubdiv(). I rewrote the recursive
	call because it caused recursion-depth errors on complicated line segments.
	"""

	while True:
		while True:
			if i >= len( sp ):
				return

			p0 = sp[i - 1][1]
			p1 = sp[i - 1][2]
			p2 = sp[i][0]
			p3 = sp[i][1]

			b = ( p0, p1, p2, p3 )

			if cspsubdiv.maxdist( b ) > flat:
				break

			i += 1

		one, two = beziersplitatt( b, 0.5 )
		sp[i - 1][2] = one[1]
		sp[i][0] = two[2]
		p = [one[2], one[3], two[1]]
		sp[i:1] = [p]




########################################################################
########################################################################
#
# The actual inkscape effect plugin


class Muralizer( inkex.Effect ):

	def __init__( self ): # MCL
		inkex.Effect.__init__( self )

		my_params = [  # param name, type, default, help
			##################################################
			# Canvas properties
			("canvasWidth" , "int", 122, "Canvas width (cm)"),
			("canvasHeight", "int", 183, "Canvas height (cm)"),

			("marginXL"    , "int",  23, "Margin X,left (cm)"),
			("marginXR"    , "int",  23, "Margin X,right (cm)"),
			("marginYT"    , "int",  23, "Margin Y,top (cm)"),


			##################################################
			# Plotter properties

			("stepsPerRev"  , "int", 48, "Steps per revolution"),
			("spoolDiameter", "int", 63, "Spool diameter, mm"),


			("smoothness", "float", 0.2, "Curve smoothing"),

			("layernumber", "int", 0, "Layer number to print"),


			##################################################
			# Manual control

			("manualType", "string", "none", "Manual command"),
			("walkDistance","int", 10, "# of steps to walk (can be negative)"),

			

			##################################################
			# Misc
			("tab", "string", "controls", 
			 "The active tab when Apply was pressed"),

			("setupType", "string", "", 
			 "The active option when Apply was pressed"),

			]

		for (p_name, p_type, p_default, p_help) in my_params:
			self.OptionParser.add_option("--%s" % p_name,
						     action="store",
						     type=p_type,
						     dest=p_name,
						     default=p_default,
						     help=p_help)
			
		self.allLayers = True
		self.plotCurrentLayer = True
		self.bPenIsUp = True
		self.virtualPenIsUp = False  #Keeps track of pen postion when stepping through plot before resuming
		self.engraverIsOn = False
		self.penDownActivatesEngraver = False
		self.fX = None
		self.fY = None
		self.fPrevX = None
		self.fPrevY = None
		self.ptFirst = None
		self.bStopped = False
		self.fSpeed = 1
		self.resumeMode = False
		self.nodeCount = int( 0 )		#NOTE: python uses 32-bit ints.
		self.nodeTarget = int( 0 )
		self.pathcount = int( 0 )
		self.LayersPlotted = 0
		self.svgSerialPort = '/dev/ttyUSB0'
		self.svgLayer = -1
		self.svgNodeCount = int( 0 )
		self.svgDataRead = False
		self.svgLastPath = int( 0 )
		self.svgLastPathNC = int( 0 )
		self.svgTotalDeltaX = int( 0 )
		self.svgTotalDeltaY = int( 0 )

		self.nDeltaX = 0
		self.nDeltaY = 0

		self.svgWidth = float( N_PAGE_WIDTH )
		self.svgHeight = float( N_PAGE_HEIGHT )
		self.svgTransform = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]

		# So that we only generate a warning once for each
		# unsupported SVG element, we use a dictionary to track
		# which elements have received a warning
		self.warnings = {}

		self.step_scaling_factor = 1

		self.ms = None # XXX ugh JBM

	def effect( self ):  # MCL
		'''Main entry point: check to see which tab is selected, and act accordingly.'''

		self.ms = MuralizerState(options=self.options)
		self.svgWidth = self.ms.page_width()
		self.svgHeight = self.ms.page_height()

		self.svg = self.document.getroot()
		self.CheckSVGforEggbotData()

		if self.options.tab == '"splash"':
			inkex.errormsg("Print!")
			self.plot()

		elif self.options.tab == '"motors"':
			inkex.errormsg("Motor properties: %s" % str(self.ms))

		elif self.options.tab == '"options"':
			inkex.errormsg("Options")

		elif self.options.tab == '"manual"':
			self.manualCommand()

		elif self.options.tab == '"layers"':
			inkex.errormsg("Layers")

		elif self.options.tab == '"Help"':
			inkex.errormsg("Halp!")

		elif self.options.tab == '"setup"':
			inkex.errormsg("Setup.")
		else:
			inkex.errormsg("Unhandled tab: %s" % self.options.tab)


		self.svgDataRead = False
		self.UpdateSVGEggbotData( self.svg )
		return


	def CheckSVGforEggbotData( self ): # MCL
		self.svgDataRead = False
		self.recursiveEggbotDataScan( self.svg )

		if ( not self.svgDataRead ):    #if there is no eggbot data, add some:
			eggbotlayer = inkex.etree.SubElement( self.svg, 'eggbot' )
			eggbotlayer.set( 'serialport', '' )
			eggbotlayer.set( 'layer', str( 0 ) )
			eggbotlayer.set( 'node', str( 0 ) )
			eggbotlayer.set( 'lastpath', str( 0 ) )
			eggbotlayer.set( 'lastpathnc', str( 0 ) )
			eggbotlayer.set( 'totaldeltax', str( 0 ) )
			eggbotlayer.set( 'totaldeltay', str( 0 ) )

	def recursiveEggbotDataScan( self, aNodeList ):  # MCL
		if self.svgDataRead:
			return

		for node in aNodeList:
			if node.tag == 'svg':
				self.recursiveEggbotDataScan( node )
			elif node.tag == inkex.addNS( 'eggbot', 'svg' ) or node.tag == 'eggbot':
				self.svgSerialPort = node.get( 'serialport' )
				self.svgLayer = int( node.get( 'layer' ) )
				self.svgNodeCount = int( node.get( 'node' ) )

				try:
					self.svgLastPath = int( node.get( 'lastpath' ) )
					self.svgLastPathNC = int( node.get( 'lastpathnc' ) )
					self.svgTotalDeltaX = int( node.get( 'totaldeltax' ) )
					self.svgTotalDeltaY = int( node.get( 'totaldeltay' ) )
					self.svgDataRead = True
				except:
					node.set( 'lastpath', str( 0 ) )
					node.set( 'lastpathnc', str( 0 ) )
					node.set( 'totaldeltax', str( 0 ) )
					node.set( 'totaldeltay', str( 0 ) )
					self.svgDataRead = True

	def UpdateSVGEggbotData( self, aNodeList ): # MCL
		if ( not self.svgDataRead ):
			for node in aNodeList:
				if node.tag == 'svg':
					self.UpdateSVGEggbotData( node )
				elif node.tag == inkex.addNS( 'eggbot', 'svg' ) or node.tag == 'eggbot':
					node.set( 'serialport', self.svgSerialPort )
					node.set( 'layer', str( self.svgLayer ) )
					node.set( 'node', str( self.svgNodeCount ) )
					node.set( 'lastpath', str( self.svgLastPath ) )
					node.set( 'lastpathnc', str( self.svgLastPathNC ) )
					node.set( 'totaldeltax', str( self.svgTotalDeltaX ) )
					node.set( 'totaldeltay', str( self.svgTotalDeltaY ) )
					self.svgDataRead = True

	def resumePlotSetup( self ):
		self.LayerFound = False
		if self.svgLayer > 0 and self.svgLayer < 101:
			self.options.layernumber = self.svgLayer
			self.allLayers = False
			self.plotCurrentLayer = False
			self.LayerFound = True
		elif self.svgLayer == 0:  # Plot all layers
			self.allLayers = True
			self.plotCurrentLayer = True
			self.LayerFound = True


		if ( self.LayerFound ):
			if ( self.svgNodeCount > 0 ):
				self.nodeTarget = self.svgNodeCount
				self.resumeMode = True
				if ( self.options.cancelOnly ):
					self.resumeMode = False
					self.fPrevX = self.svgTotalDeltaX
					self.fPrevY = self.svgTotalDeltaY

					self.fX = 0
					self.fY = 0

					self.plotLineAndTime()

					self.ms.home()

					self.ms.cmd_pen_up()   #Always end with pen-up
					self.svgLayer = 0
					self.svgNodeCount = 0
					self.svgLastPath = 0
					self.svgLastPathNC = 0
					self.svgTotalDeltaX = 0
					self.svgTotalDeltaY = 0

	def manualCommand( self ): # MCL
		"""Execute commands from the "manual" tab"""


		dispatch = {
			"none": lambda: self.ms.cmd_nop(),
			"enable-motors": lambda: self.ms.cmd_enable_motors(),
			"disable-motors": lambda: self.ms.cmd_disable_motors(),
			"walk-r0": lambda: self.ms.cmd_move_r0(self.options.walkDistance),
			"walk-r1": lambda: self.ms.cmd_move_r1(self.options.walkDistance),
			"version-check": lambda: self.ms.cmd_version(),
			}


		if self.options.manualType not in dispatch:
			inkex.errormsg("Unknown manual command to dispatch: %s" % self.options.manualType)
			return

		serial_fd = None

		try:
			retval = dispatch[self.options.manualType]()

		finally:
			if serial_fd:
				serial_fd.close()
				self.ms.detach_serial()
			self.ms.cmd_scram() # This should work even if the serial port is gone



		if None != retval:
			inkex.errormsg("Command: %s, result: %s" % (self.options.manualType, str(retval)))




	def setupCommand( self ): # MIP
		"""Execute commands from the "setup" tab"""
		self.ms.ensure_setup(self.options)

		if self.options.setupType == "align-mode":
			self.ms.cmd_pen_up()
			self.ms.cmd_disable_motors()

		elif self.options.setupType == "toggle-pen":
			self.ms.cmd_pen_toggle()


	def debugNote(self, s):
		self.ms.alert(s)


	def getFitScale(self):
		svgHeight = self.getLength('height', self.ms.page_height())
		svgWidth = self.getLength('width', self.ms.page_width())

		scale_h = self.ms.page_height() / svgHeight
		scale_w = self.ms.page_width() / svgWidth

		self.debugNote("Scale: svg: %.1f x %.1f; page: %.1f x %.1f; scales: %.3f / %.3f" % (svgWidth, svgHeight, self.ms.page_width(), self.ms.page_height(), scale_w, scale_h))

		return min(scale_h, scale_w)

		
		

	def plot( self ): # MIP
		'''Perform the actual plotting, if selected in the interface:'''
		#parse the svg data as a series of line segments and send each segment to be plotted

		inkex.errormsg("Layers to print: " + str(self.svgLayer))

		self.debugNote("Starting plot.  Layers to print: %s" % (str(self.svgLayer)))



		# Viewbox handling
		# Also ignores the preserveAspectRatio attribute
		viewbox = self.svg.get( 'viewBox' )
		if viewbox:
			vinfo = viewbox.strip().replace( ',', ' ' ).split( ' ' )
			if ( vinfo[2] != 0 ) and ( vinfo[3] != 0 ):
				sx = self.svgWidth / float( vinfo[2] )
				sy = self.svgHeight / float( vinfo[3] )
				xform_scale = 'scale(%f,%f)' % (sx,sy)

				dx = -float(vinfo[0])
				dy = -float(vinfo[1])
				xform_xlate = 'translate(%f,%f)' % (dx,dy)

				xform = " ".join([xform_scale, xform_xlate])

				self.svgTransform = parseTransform(xform)
		else:
			scalefactor = self.getFitScale()
			xform = "scale(%f,%f)" % (scalefactor, scalefactor)
			self.svgTransform = parseTransform(xform)

		self.debugNote("svgTransform: %s" % self.svgTransform)


		serial_fd = None

		try:

			self.recursivelyTraverseSvg(self.svg, self.svgTransform)
			
			inkex.errormsg('Final node count: ' + str(self.svgNodeCount))
			self.debugNote('Final node count: ' + str(self.svgNodeCount))

			if ( not self.bStopped ):
				self.svgLayer = 0
				self.svgNodeCount = 0
				self.svgLastPath = 0
				self.svgLastPathNC = 0
				self.svgTotalDeltaX = 0
				self.svgTotalDeltaY = 0

		finally:
			if serial_fd:
				serial_fd.close()
				self.ms.detach_serial()
			self.ms.cmd_scram() # This should work even if the serial port is gone

	def recursivelyTraverseSvg( self, aNodeList,
			matCurrent=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
			parent_visibility='visible' ):
		"""
		Recursively traverse the svg file to plot out all of the
		paths.  The function keeps track of the composite transformation
		that should be applied to each path.

		This function handles path, group, line, rect, polyline, polygon,
		circle, ellipse and use (clone) elements.  Notable elements not
		handled include text.  Unhandled elements should be converted to
		paths in Inkscape.
		"""
		for node in aNodeList:
			# Ignore invisible nodes
			v = node.get( 'visibility', parent_visibility )
			if v == 'inherit':
				v = parent_visibility
			if v == 'hidden' or v == 'collapse':
				pass

			# first apply the current matrix transform to this node's tranform
			matNew = composeTransform( matCurrent, parseTransform( node.get( "transform" ) ) )

			if node.tag == inkex.addNS( 'g', 'svg' ) or node.tag == 'g':

				self.ms.cmd_pen_up()
				if ( node.get( inkex.addNS( 'groupmode', 'inkscape' ) ) == 'layer' ):
					if not self.allLayers:
						#inkex.errormsg('Plotting layer named: ' + node.get(inkex.addNS('label', 'inkscape')))
						self.DoWePlotLayer( node.get( inkex.addNS( 'label', 'inkscape' ) ) )
				self.recursivelyTraverseSvg( node, matNew, parent_visibility=v )

			elif node.tag == inkex.addNS( 'use', 'svg' ) or node.tag == 'use':

				# A <use> element refers to another SVG element via an xlink:href="#blah"
				# attribute.  We will handle the element by doing an XPath search through
				# the document, looking for the element with the matching id="blah"
				# attribute.  We then recursively process that element after applying
				# any necessary (x,y) translation.
				#
				# Notes:
				#  1. We ignore the height and width attributes as they do not apply to
				#     path-like elements, and
				#  2. Even if the use element has visibility="hidden", SVG still calls
				#     for processing the referenced element.  The referenced element is
				#     hidden only if its visibility is "inherit" or "hidden".

				refid = node.get( inkex.addNS( 'href', 'xlink' ) )
				if refid:
					# [1:] to ignore leading '#' in reference
					path = '//*[@id="%s"]' % refid[1:]
					refnode = node.xpath( path )
					if refnode:
						x = float( node.get( 'x', '0' ) )
						y = float( node.get( 'y', '0' ) )
						# Note: the transform has already been applied
						if ( x != 0 ) or (y != 0 ):
							matNew2 = composeTransform( matNew, parseTransform( 'translate(%f,%f)' % (x,y) ) )
						else:
							matNew2 = matNew
						v = node.get( 'visibility', v )
						self.recursivelyTraverseSvg( refnode, matNew2, parent_visibility=v )
					else:
						pass
				else:
					pass

			elif node.tag == inkex.addNS( 'path', 'svg' ):

				self.pathcount += 1

				# if we're in resume mode AND self.pathcount < self.svgLastPath,
				#    then skip over this path.
				# if we're in resume mode and self.pathcount = self.svgLastPath,
				#    then start here, and set
				# self.nodeCount equal to self.svgLastPathNC
				if self.resumeMode and ( self.pathcount == self.svgLastPath ):
					self.nodeCount = self.svgLastPathNC
				if self.resumeMode and ( self.pathcount < self.svgLastPath ):
					pass
				else:
					self.plotPath( node, matNew )
					if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
						self.svgLastPath += 1
						self.svgLastPathNC = self.nodeCount

			elif node.tag == inkex.addNS( 'rect', 'svg' ) or node.tag == 'rect':

				# Manually transform
				#
				#    <rect x="X" y="Y" width="W" height="H"/>
				#
				# into
				#
				#    <path d="MX,Y lW,0 l0,H l-W,0 z"/>
				#
				# I.e., explicitly draw three sides of the rectangle and the
				# fourth side implicitly

				self.pathcount += 1
				# if we're in resume mode AND self.pathcount < self.svgLastPath,
				#    then skip over this path.
				# if we're in resume mode and self.pathcount = self.svgLastPath,
				#    then start here, and set
				# self.nodeCount equal to self.svgLastPathNC
				if self.resumeMode and ( self.pathcount == self.svgLastPath ):
					self.nodeCount = self.svgLastPathNC
				if self.resumeMode and ( self.pathcount < self.svgLastPath ):
					pass
				else:
					# Create a path with the outline of the rectangle
					newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
					x = float( node.get( 'x' ) )
					y = float( node.get( 'y' ) )
					w = float( node.get( 'width' ) )
					h = float( node.get( 'height' ) )
					s = node.get( 'style' )
					if s:
						newpath.set( 'style', s )
					t = node.get( 'transform' )
					if t:
						newpath.set( 'transform', t )
					a = []
					a.append( ['M ', [x, y]] )
					a.append( [' l ', [w, 0]] )
					a.append( [' l ', [0, h]] )
					a.append( [' l ', [-w, 0]] )
					a.append( [' Z', []] )
					newpath.set( 'd', simplepath.formatPath( a ) )
					self.plotPath( newpath, matNew )

			elif node.tag == inkex.addNS( 'line', 'svg' ) or node.tag == 'line':

				# Convert
				#
				#   <line x1="X1" y1="Y1" x2="X2" y2="Y2/>
				#
				# to
				#
				#   <path d="MX1,Y1 LX2,Y2"/>

				self.pathcount += 1
				# if we're in resume mode AND self.pathcount < self.svgLastPath,
				#    then skip over this path.
				# if we're in resume mode and self.pathcount = self.svgLastPath,
				#    then start here, and set
				# self.nodeCount equal to self.svgLastPathNC

				if self.resumeMode and ( self.pathcount == self.svgLastPath ):
					self.nodeCount = self.svgLastPathNC
				if self.resumeMode and ( self.pathcount < self.svgLastPath ):
					pass
				else:
					# Create a path to contain the line
					newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
					x1 = float( node.get( 'x1' ) )
					y1 = float( node.get( 'y1' ) )
					x2 = float( node.get( 'x2' ) )
					y2 = float( node.get( 'y2' ) )
					s = node.get( 'style' )
					if s:
						newpath.set( 'style', s )
					t = node.get( 'transform' )
					if t:
						newpath.set( 'transform', t )
					a = []
					a.append( ['M ', [x1, y1]] )
					a.append( [' L ', [x2, y2]] )
					newpath.set( 'd', simplepath.formatPath( a ) )
					self.plotPath( newpath, matNew )
					if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
						self.svgLastPath += 1
						self.svgLastPathNC = self.nodeCount

			elif node.tag == inkex.addNS( 'polyline', 'svg' ) or node.tag == 'polyline':

				# Convert
				#
				#  <polyline points="x1,y1 x2,y2 x3,y3 [...]"/>
				#
				# to
				#
				#   <path d="Mx1,y1 Lx2,y2 Lx3,y3 [...]"/>
				#
				# Note: we ignore polylines with no points

				pl = node.get( 'points', '' ).strip()
				if pl == '':
					pass

				self.pathcount += 1
				#if we're in resume mode AND self.pathcount < self.svgLastPath, then skip over this path.
				#if we're in resume mode and self.pathcount = self.svgLastPath, then start here, and set
				# self.nodeCount equal to self.svgLastPathNC

				if self.resumeMode and ( self.pathcount == self.svgLastPath ):
					self.nodeCount = self.svgLastPathNC

				if self.resumeMode and ( self.pathcount < self.svgLastPath ):
					pass

				else:
					pa = pl.split()
					if not len( pa ):
						pass
					# Issue 29: pre 2.5.? versions of Python do not have
					#    "statement-1 if expression-1 else statement-2"
					# which came out of PEP 308, Conditional Expressions
					#d = "".join( ["M " + pa[i] if i == 0 else " L " + pa[i] for i in range( 0, len( pa ) )] )
					d = "M " + pa[0]
					for i in range( 1, len( pa ) ):
						d += " L " + pa[i]
					newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
					newpath.set( 'd', d );
					s = node.get( 'style' )
					if s:
						newpath.set( 'style', s )
					t = node.get( 'transform' )
					if t:
						newpath.set( 'transform', t )
					self.plotPath( newpath, matNew )
					if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
						self.svgLastPath += 1
						self.svgLastPathNC = self.nodeCount

			elif node.tag == inkex.addNS( 'polygon', 'svg' ) or node.tag == 'polygon':

				# Convert
				#
				#  <polygon points="x1,y1 x2,y2 x3,y3 [...]"/>
				#
				# to
				#
				#   <path d="Mx1,y1 Lx2,y2 Lx3,y3 [...] Z"/>
				#
				# Note: we ignore polygons with no points

				pl = node.get( 'points', '' ).strip()
				if pl == '':
					pass

				self.pathcount += 1
				#if we're in resume mode AND self.pathcount < self.svgLastPath, then skip over this path.
				#if we're in resume mode and self.pathcount = self.svgLastPath, then start here, and set
				# self.nodeCount equal to self.svgLastPathNC

				if self.resumeMode and ( self.pathcount == self.svgLastPath ):
					self.nodeCount = self.svgLastPathNC

				if self.resumeMode and ( self.pathcount < self.svgLastPath ):
					pass

				else:
					pa = pl.split()
					if not len( pa ):
						pass
					# Issue 29: pre 2.5.? versions of Python do not have
					#    "statement-1 if expression-1 else statement-2"
					# which came out of PEP 308, Conditional Expressions
					#d = "".join( ["M " + pa[i] if i == 0 else " L " + pa[i] for i in range( 0, len( pa ) )] )
					d = "M " + pa[0]
					for i in range( 1, len( pa ) ):
						d += " L " + pa[i]
					d += " Z"
					newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
					newpath.set( 'd', d );
					s = node.get( 'style' )
					if s:
						newpath.set( 'style', s )
					t = node.get( 'transform' )
					if t:
						newpath.set( 'transform', t )
					self.plotPath( newpath, matNew )
					if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
						self.svgLastPath += 1
						self.svgLastPathNC = self.nodeCount

			elif node.tag == inkex.addNS( 'ellipse', 'svg' ) or \
				node.tag == 'ellipse' or \
				node.tag == inkex.addNS( 'circle', 'svg' ) or \
				node.tag == 'circle':

					# Convert circles and ellipses to a path with two 180 degree arcs.
					# In general (an ellipse), we convert
					#
					#   <ellipse rx="RX" ry="RY" cx="X" cy="Y"/>
					#
					# to
					#
					#   <path d="MX1,CY A RX,RY 0 1 0 X2,CY A RX,RY 0 1 0 X1,CY"/>
					#
					# where
					#
					#   X1 = CX - RX
					#   X2 = CX + RX
					#
					# Note: ellipses or circles with a radius attribute of value 0 are ignored

					if node.tag == inkex.addNS( 'ellipse', 'svg' ) or node.tag == 'ellipse':
						rx = float( node.get( 'rx', '0' ) )
						ry = float( node.get( 'ry', '0' ) )
					else:
						rx = float( node.get( 'r', '0' ) )
						ry = rx
					if rx == 0 or ry == 0:
						pass

					self.pathcount += 1
					#if we're in resume mode AND self.pathcount < self.svgLastPath, then skip over this path.
					#if we're in resume mode and self.pathcount = self.svgLastPath, then start here, and set
					# self.nodeCount equal to self.svgLastPathNC

					if self.resumeMode and ( self.pathcount == self.svgLastPath ):
						self.nodeCount = self.svgLastPathNC

					if self.resumeMode and ( self.pathcount < self.svgLastPath ):
						pass

					else:
						cx = float( node.get( 'cx', '0' ) )
						cy = float( node.get( 'cy', '0' ) )
						x1 = cx - rx
						x2 = cx + rx
						d = 'M %f,%f ' % ( x1, cy ) + \
							'A %f,%f ' % ( rx, ry ) + \
							'0 1 0 %f,%f ' % ( x2, cy ) + \
							'A %f,%f ' % ( rx, ry ) + \
							'0 1 0 %f,%f' % ( x1, cy )
						newpath = inkex.etree.Element( inkex.addNS( 'path', 'svg' ) )
						newpath.set( 'd', d );
						s = node.get( 'style' )
						if s:
							newpath.set( 'style', s )
						t = node.get( 'transform' )
						if t:
							newpath.set( 'transform', t )
						self.plotPath( newpath, matNew )
						if ( not self.bStopped ):	#an "index" for resuming plots quickly-- record last complete path
							self.svgLastPath += 1
							self.svgLastPathNC = self.nodeCount
			elif node.tag == inkex.addNS( 'metadata', 'svg' ) or node.tag == 'metadata':
				pass
			elif node.tag == inkex.addNS( 'defs', 'svg' ) or node.tag == 'defs':
				pass
			elif node.tag == inkex.addNS( 'namedview', 'sodipodi' ) or node.tag == 'namedview':
				pass
			elif node.tag == inkex.addNS( 'eggbot', 'svg' ) or node.tag == 'eggbot':
				pass
			elif node.tag == inkex.addNS( 'title', 'svg' ) or node.tag == 'title':
				pass
			elif node.tag == inkex.addNS( 'desc', 'svg' ) or node.tag == 'desc':
				pass
			elif node.tag == inkex.addNS( 'text', 'svg' ) or node.tag == 'text':
				if not self.warnings.has_key( 'text' ):
					inkex.errormsg( gettext.gettext( 'Warning: unable to draw text; ' +
						'please convert it to a path first.  Consider using the ' +
						'Hershey Text extension which is located under the '+
						'"Render" category of extensions.' ) )
					self.warnings['text'] = 1
				pass
			elif node.tag == inkex.addNS( 'image', 'svg' ) or node.tag == 'image':
				if not self.warnings.has_key( 'image' ):
					inkex.errormsg( gettext.gettext( 'Warning: unable to draw bitmap images; ' +
						'please convert them to line art first.  Consider using the "Trace bitmap..." ' +
						'tool of the "Path" menu.  Mac users please note that some X11 settings may ' +
						'cause cut-and-paste operations to paste in bitmap copies.' ) )
					self.warnings['image'] = 1
				pass
			elif node.tag == inkex.addNS( 'pattern', 'svg' ) or node.tag == 'pattern':
				pass
			elif node.tag == inkex.addNS( 'radialGradient', 'svg' ) or node.tag == 'radialGradient':
				# Similar to pattern
				pass
			elif node.tag == inkex.addNS( 'linearGradient', 'svg' ) or node.tag == 'linearGradient':
				# Similar in pattern
				pass
			elif node.tag == inkex.addNS( 'style', 'svg' ) or node.tag == 'style':
				# This is a reference to an external style sheet and not the value
				# of a style attribute to be inherited by child elements
				pass
			elif node.tag == inkex.addNS( 'cursor', 'svg' ) or node.tag == 'cursor':
				pass
			elif node.tag == inkex.addNS( 'color-profile', 'svg' ) or node.tag == 'color-profile':
				# Gamma curves, color temp, etc. are not relevant to single color output
				pass
			elif not isinstance( node.tag, basestring ):
				# This is likely an XML processing instruction such as an XML
				# comment.  lxml uses a function reference for such node tags
				# and as such the node tag is likely not a printable string.
				# Further, converting it to a printable string likely won't
				# be very useful.
				pass
			else:
				if not self.warnings.has_key( str( node.tag ) ):
					t = str( node.tag ).split( '}' )
					inkex.errormsg( gettext.gettext( 'Warning: unable to draw <' + str( t[-1] ) +
						'> object, please convert it to a path first.' ) )
					self.warnings[str( node.tag )] = 1
				pass

	def DoWePlotLayer( self, strLayerName ):
		"""
		We are only plotting *some* layers. Check to see
		whether or not we're going to plot this one.

		First: scan first 4 chars of node id for first non-numeric character,
		and scan the part before that (if any) into a number

		Then, see if the number matches the layer.
		"""

		TempNumString = 'x'
		stringPos = 1
		CurrentLayerName = string.lstrip( strLayerName ) #remove leading whitespace

		# Look at layer name.  Sample first character, then first two, and
		# so on, until the string ends or the string no longer consists of
		# digit characters only.

		MaxLength = len( CurrentLayerName )
		if MaxLength > 0:
			while stringPos <= MaxLength:
				if str.isdigit( CurrentLayerName[:stringPos] ):
					TempNumString = CurrentLayerName[:stringPos] # Store longest numeric string so far
					stringPos = stringPos + 1
				else:
					break

		self.plotCurrentLayer = False    #Temporarily assume that we aren't plotting the layer
		if ( str.isdigit( TempNumString ) ):
			if ( self.svgLayer == int( float( TempNumString ) ) ):
				self.plotCurrentLayer = True	#We get to plot the layer!
				self.LayersPlotted += 1
		#Note: this function is only called if we are NOT plotting all layers.

	def getLength( self, name, default ): #MCL
		'''
		Get the <svg> attribute with name "name" and default value "default"
		Parse the attribute into a value and associated units.  Then, accept
		no units (''), units of pixels ('px'), and units of percentage ('%').
		'''
		str = self.svg.get( name )
		if str:
			v, u = parseLengthWithUnits( str )
			if not v:
				# Couldn't parse the value
				return None
			elif ( u == '' ) or ( u == 'px' ):
				return v
			elif u == '%':
				return float( default ) * v / 100.0
			else:
				# Unsupported units
				return None
		else:
			# No width specified; assume the default value
			return float( default )

	def getDocProps( self ): # MCL
		'''
		Get the document's height and width attributes from the <svg> tag.
		Use a default value in case the property is not present or is
		expressed in units of percentages.
		'''
		self.svgHeight = self.getLength( 'height', self.ms.page_width() )
		self.svgWidth = self.getLength( 'width', self.ms.page_height() )
		if ( self.svgHeight == None ) or ( self.svgWidth == None ):
			return False
		else:
			return True

	def plotPath( self, path, matTransform ): # MIP
		'''
		Plot the path while applying the transformation defined
		by the matrix [matTransform].
		'''
		# turn this path into a cubicsuperpath (list of beziers)...

		d = path.get( 'd' )

		if len( simplepath.parsePath( d ) ) == 0:
			return

		p = cubicsuperpath.parsePath( d )

		# ...and apply the transformation to each point
		applyTransformToPath( matTransform, p )

		# p is now a list of lists of cubic beziers [control pt1, control pt2, endpoint]
		# where the start-point is the last point in the previous segment.
		for sp in p:

			subdivideCubicPath( sp, self.options.smoothness )
			nIndex = 0

			for csp in sp:
				if self.bStopped:
					return

				if self.plotCurrentLayer:
					if nIndex == 0: # Pen up to start of curve
						self.ms.cmd_pen_up()
						self.virtualPenIsUp = True
					elif nIndex == 1: # Pen down to the end of the curve
						self.ms.cmd_pen_down()
						self.virtualPenIsUp = False

				nIndex += 1

				self.fX = float( csp[1][0] )
				self.fY = float( csp[1][1] )

				# Precondition: where the heck are we coming from?
				if self.ptFirst is None:
					self.fPrevX = self.ms.area_x()
					self.fPrevY = self.ms.area_y()

					self.ptFirst = (self.fPrevX, self.fPrevY)


				if self.plotCurrentLayer:
					self.plotLineAndTime()
					self.fPrevX = self.fX
					self.fPrevY = self.fY

			self.ms.cmd_pen_up() # Raise the pen at the end of each curve

	def doTimedPause( self, nPause ):
		while ( nPause > 0 ):
			if ( nPause > 750 ):
				td = int( 750 )
			else:
				td = nPause
				if ( td < 1 ):
					td = int( 1 ) # don't allow zero-time moves
			if ( not self.resumeMode ):
				self.ms.cmd_pause(td)
			nPause -= td

	def stop( self ):
		self.bStopped = True

	def plotLineAndTime( self ): # MIP
		'''
		Send commands out the com port as a line segment (dx, dy) and a time (ms) the segment
		should take to implement
		'''

		if self.bStopped:
			return
		if ( self.fPrevX is None ):
			return

		self.nDeltaX = int( self.fX ) - int( self.fPrevX )
		self.nDeltaY = int( self.fY ) - int( self.fPrevY )


		if ( distance( self.nDeltaX, self.nDeltaY ) > 0 ):
			self.nodeCount += 1

			if self.resumeMode:
				if ( self.nodeCount > self.nodeTarget ):
					self.resumeMode = False
					#inkex.errormsg('First node plotted will be number: ' + str(self.nodeCount))
					if ( not self.virtualPenIsUp ):
						self.ms.cmd_pen_down()

			###############################
			# Actual motion control

			if not self.resumeMode:
				self.ms.go_to_area(self.fX, self.fY)

			####################
			# Check if we need to pause
			# 
			if self.ms.cmd_button_down():
				self.svgNodeCount = self.nodeCount;
				inkex.errormsg( 'Plot paused by button press after segment number ' + str( self.nodeCount ) + '.' )
				inkex.errormsg( 'Use the "resume" feature to continue.' )
				self.engraverOff()
				self.bStopped = True
				return

def distance( x, y ):
	'''
	Pythagorean theorem!
	'''
	return sqrt( x * x + y * y )

e = Muralizer()
e.affect()
