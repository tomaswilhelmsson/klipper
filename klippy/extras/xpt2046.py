# XPT2046 resisitive touch screen controller
#
# Copyright (_c) 2020 Martijn van Buul <martijn.van.buul@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import struct
from . import bus
import logging

REPEAT_BACKOFF = 0.5
REPEAT_INTERVAL = 0.25
LONGCLICK_INTERVAL = 1

# http://www.grix.it/forum/forum_thread.php?id_forum=3&id_thread=506128
XPT_CTRL_PD0 = (1 << 0)
XPT_CTRL_PD1 = (1 << 1)
XPT_CTRL_DFR = (1 << 2)
XPT_CTRL_EIGHT_BIT = (1 << 3)
XPT_CTRL_START = (1 << 7)
XPT_CTRL_SWITCH_SHIFT = 4

#0b10010011
XPT_CMD_X_POS = ((1 << XPT_CTRL_SWITCH_SHIFT) | XPT_CTRL_START | XPT_CTRL_PD0 | XPT_CTRL_PD1)
#0b11010011
XPT_CMD_Y_POS = ((5 << XPT_CTRL_SWITCH_SHIFT) | XPT_CTRL_START | XPT_CTRL_PD0 | XPT_CTRL_PD1)

XPT_ENABLE_PENIRQ = ((1 << XPT_CTRL_SWITCH_SHIFT) | XPT_CTRL_START)

class XptConfigException(Exception):
    def __init__(self, value):
        super(XptConfigException, self).__init__(value)
        self.value = value

def _parse_coordinate(coordinate_string):
    components = coordinate_string.strip().split(',')

    try:
        pair = tuple(math.floor(float(p.strip())) for p in components)
    except ValueError:
        raise XptConfigException(
            "Malformed value '%s'" % (coordinate_string,))

    if len(pair) != 2:
        raise XptConfigException(
            "Malformed value '%s'" % (coordinate_string,))

    return pair

class Xpt2046MenuAction(object):
    def __init__(self, menu, function):
        self._menu = menu
        self._function = function

    def invoke(self, eventtime):
        self._menu.key_event(self._function, eventtime)

class Xpt2046GcodeAction(object):
    def __init__(self, printer, config, template_name):
        gcode_macro = printer.load_object(config, 'gcode_macro')
        self._gcode = printer.lookup_object("gcode")
        self._template = gcode_macro.load_template(config, template_name)

    def invoke(self, eventtime):
        del eventtime
        self._gcode.run_script(self._template.render())

class Xpt2046DummyAction(object):
    # pylint: disable=R0201
    def invoke(self, eventtime):
        del eventtime

def _create_action(printer_object, config, action_prefix):
    menu_function = config.get(action_prefix + '_function', None)
    if menu_function:
        menu_object = printer_object.lookup_object("menu")
        return Xpt2046MenuAction(menu_object, menu_function)
    if config.get(action_prefix + '_gcode', None):
        return Xpt2046GcodeAction(printer_object, config,
                                  action_prefix + '_gcode')
    return None

class Xpt2046Button(object):
    def __init__(self, points):
        self.points = []

        max_x = None
        points = points.split('\n')
        for point in points:
            if point:
                new_point = _parse_coordinate(point)
                if max_x is None or max_x < new_point[0]:
                    max_x = new_point[0]

                self.points.append(new_point)

        if len(self.points) < 3:
            raise XptConfigException(
                "Polygon with at least 3 vertices expected")

        self._max_ray_x = max_x + 100 # X coordinate used for ray casting

    def check_xy(self, location):
        def ccw(_a, _b, _c):
            return ((_c[1]-_a[1]) * (_b[0]-_a[0]) >
                    (_b[1]-_a[1]) * (_c[0]-_a[0]))

        def intersect(_a, _b, _c, _d):
            return (ccw(_a, _c, _d) != ccw(_b, _c, _d) and
                    ccw(_a, _b, _c) != ccw(_a, _b, _d))

        result = False
        # use a 0.1 offset (while polygon points are truncated) to ensure
        # that the casted ray can never be colinear with a polygon vertex
        outside_location = (self._max_ray_x, location[1] + 0.1)
        for i in range(len(self.points)-1):
            if intersect(self.points[i], self.points[i+1],
                         location, outside_location):
                result = not result

        if intersect(self.points[-1], self.points[0],
                     location, outside_location):
            result = not result

        return result

    # The following two methods are to be overridden by derived classes

    # Called when a button is clicked. Can return a timestamp for an expected
    # repeat event.
    def clicked(self, eventtime):
        pass

    # Called repeatedly while a button is being held. Can return a timestamp
    # for the next expected repeat event.
    def repeat(self, eventtime):
        pass

class Xpt2046OneshotButton(Xpt2046Button):
    def __init__(self, points, action):
        super(Xpt2046OneshotButton, self).__init__(points)
        self._action = action

    #override
    def clicked(self, eventtime):
        self._action.invoke(eventtime)
        return None

class Xpt2046RepeatingButton(Xpt2046Button):
    def __init__(self, points, action, repeat_action):
        super(Xpt2046RepeatingButton, self).__init__(points)
        self._action = action
        self._repeat_action = repeat_action

    #override
    def clicked(self, eventtime):
        self._action.invoke(eventtime)
        return eventtime + REPEAT_BACKOFF

    #override
    def repeat(self, eventtime):
        self._repeat_action.invoke(eventtime)
        return eventtime + REPEAT_INTERVAL

class Xpt2046LongclickButton(Xpt2046Button):
    def __init__(self, points, action, longclick_action):
        super(Xpt2046LongclickButton, self).__init__(points)

        self._action = action
        self._longclick_action = longclick_action

    #override
    def clicked(self, eventtime):
        self._action.invoke(eventtime)
        return eventtime + LONGCLICK_INTERVAL

    #override
    def repeat(self, eventtime):
        self._longclick_action.invoke(eventtime)
        return None # no more callbacks required



class Xpt2046(object):
    def __init__(self, config):
        printer = config.get_printer()
        self._reactor = printer.get_reactor()
        self._gcode = printer.lookup_object("gcode")
        self._repeat_timer = self._reactor.register_timer(self._repeat_event)
        self._active_button = None
        self._spi = bus.MCU_SPI_from_config(
            config, 0, pin_option='cs_pin', default_speed=25000)

        button_pin = config.get('penirq_pin')
        buttons_object = printer.lookup_object("buttons")

        buttons_object.register_buttons([button_pin], self._penirq_callback)

        self.buttons = []

        self.busy = False

        # Discover soft buttons
        for i in range(999):
            button_prefix = 'button%d' % (i,)

            func = _create_action(printer, config, button_prefix)
            if not func:
                func = Xpt2046DummyAction()

            repeat_func = _create_action(printer, config,
                                         button_prefix + '_repeat')

            long_func = _create_action(printer, config,
                                       button_prefix + '_longpress')

            button_points = config.get(button_prefix + '_points', None)

            if not button_points or not (repeat_func or long_func or func):
                break

            try:
                if repeat_func and long_func:
                    raise XptConfigException(
                        "repeat_(function/button) and long_(function/button) "
                        "cannot be defined at the same time")
                elif repeat_func:
                    self.buttons.append(Xpt2046RepeatingButton(
                        button_points, func, repeat_func))
                elif long_func:
                    self.buttons.append(Xpt2046LongclickButton(
                        button_points, func, long_func))
                else:
                    self.buttons.append(Xpt2046OneshotButton(
                        button_points, func))

            except XptConfigException as err:
                raise config.error(
                    "Error while parsing xp2046 button %d: %s"
                    %(i, err.value))

        self._gcode.register_command(
            "XPT_TOUCH_REPORT", self._cmd_touch_report,
            desc="Enable/disable touch panel reporting")
        self.report_enabled = False

        printer.register_event_handler("klippy:ready", self._handle_ready)

    def _penirq_callback(self, eventtime, state):
        if state:
            result = self._get_touch_xy()
            if None in result: # Check if there is a None result, bad reading etc
              return
            #logging.info("Touch point x: %s, y: %s", result[0], result[1])
            if self.report_enabled:
                self._gcode.respond_info("XPT2046 touch: (%d,%d)" % result)

            for button_candidate in self.buttons:
                if button_candidate.check_xy(result):
                    self._active_button = button_candidate
                    desired_callback = button_candidate.clicked(eventtime)
                    if desired_callback:
                        self._reactor.update_timer(self._repeat_timer,
                                                   desired_callback)
                    break
        else:
            if self.report_enabled:
                self._gcode.respond_info("XPT2046 release")

            self._active_button = None

    def _repeat_event(self, eventtime):
        desired_callback = None
        if self._active_button:
            desired_callback = self._active_button.repeat(eventtime)

        return desired_callback or self._reactor.NEVER

    def _cmd_touch_report(self, gcmd):
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.report_enabled = (enable != 0)
        else:
            self.report_enabled = not self.report_enabled

        gcmd.respond_info("XPT2046 touch reporting has been %s"
                          %("enabled" if self.report_enabled else "disabled"))

    def _handle_ready(self):
        logging.info("init xpt touch panel")
        # Ensure chip is in the correct powerdown mode.
        #self._spi.spi_send([0xD0, 0x00])
        self._spi.spi_send([XPT_ENABLE_PENIRQ])
        #self._spi.spi_send([CTRL_HI_Y | CTRL_LO_DFR, 0x00])
    
    def stdev(self, value_tuples):
        x, y = zip(*value_tuples)
        n = len(value_tuples)
        x_mean = sum(x) / n
        y_mean = sum(y) / n
        x_var = sum((_x - x_mean) ** 2 for _x in x) / n
        y_var = sum((_y - y_mean) ** 2 for _y in y) / n
        x_dev = x_var ** 0.5
        y_dev = y_var ** 0.5
    
        return (x_dev, y_dev)

    def mean(self, value_tuples):
        n = len(value_tuples)
        x, y = zip(*value_tuples)
        
        return (sum(x) / n, sum(y) / n)

    # takes tuples of (x, y) values
    def filter_data(self, data, limit_multiplier = 1.3):
        try:
          deviations = self.stdev(data)
          means = self.mean(data)        
        except ValueError as e:
          logging.exception(e)
          return (-1, -1)

        anomaly_cut_off_x = deviations[0] * limit_multiplier
        anomaly_cut_off_y = deviations[0] * limit_multiplier
        anomaly_cut_off_y = deviations[1] * limit_multiplier
        anomaly_cut_off_y = deviations[1] * limit_multiplier

        
        x_lower_limit = means[0] - anomaly_cut_off_x
        x_upper_limit = means[0] + anomaly_cut_off_x
        y_lower_limit = means[1] - anomaly_cut_off_y
        y_upper_limit = means[1] + anomaly_cut_off_y
        
        ok_data = []
        
        for point in data:
            if point[0] <= x_upper_limit and \
              point[0] >= x_lower_limit and \
              point[1] <= y_upper_limit and \
              point[1] >= y_lower_limit:
                ok_data.append(point)
        
        if len(ok_data) == 0:
          return None, None

        return self.mean(ok_data)
        
    def _get_touch_xy(self):
        # logging.info("get position touch")
        try:
          positions = []
          #x = [None] * 3
          #y = [None] * 3
          for i in range(3):
            # B1 (BBBB BBBB) 
            # B2 (BBBB BPPP) Byte 2 incomming data is padded by 3 bits
            # B3 = B1 << 8 | B2
            # B3 (BBBB BBBB BBBB B000)
            # B4 B3 >> 3
            # B4 (000B BBBB BBBB BBBB)
            #self._spi.spi_transfer([XPT_CMD_X_POS, 0x00, 0x00, XPT_CMD_X_POS, 0x00, 0x00])
            payload = [XPT_CMD_X_POS, 0x00, 0x00]
            resp = self._spi.spi_transfer(payload)['response']
            x_pos = (ord(resp[1]) << 8 | ord(resp[2])) >> 3
            #x[i] = x_pos
            
            #self._spi.spi_send([XPT_CMD_Y_POS])
            payload = [XPT_CMD_Y_POS, 0x00, 0x00]
            resp = self._spi.spi_transfer(payload)['response']
            y_pos = (ord(resp[1]) << 8 | ord(resp[2])) >> 3
            #y[i] = y_pos

            positions.append((y_pos, x_pos))

            logging.info("sample: %s, x: %s, y: %s", i, x_pos, y_pos)
        except Exception as e: # KeyError it seems
          logging.info(e)
          return None
        finally:
          self._spi.spi_send([XPT_ENABLE_PENIRQ])

        _x, _y = self.filter_data(positions)

        logging.info("Touch position x: %s, y: %s", _x, _y)
        return (_x, _y)

def load_config_prefix(config):
    return Xpt2046(config)
