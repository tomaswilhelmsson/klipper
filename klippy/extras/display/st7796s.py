# Support for ST7796S (240x320 graphics) LCD displays
#
# Copyright (C) 2020  Martijn van Buul <martijn.van.buul@gmail.com>
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

# https://www.displayfuture.com/Display/datasheet/controller/ST7796s.pdf

# python ./klippy/klippy.py ~/printer.cfg -o test.serial -v -d out/klipper.dict
# python ./klippy/parsedump.py out/klipper.dict test.serial > test.txt

import math
from . import font8x14
from .. import bus
import logging
import time

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

# Recovery time after reset
ST7796_RESET_DELAY = 0.200 # min. 150 ms according to manual
ST7796_MINIMAL_DELAY = 0.100 # min. 5 ms according to manual

MAX_BYTES_IN_REQUEST = 48      # max payload to encode in a single request.
MAX_BYTES_TO_KEEP_PENDING = 45 # max payload to keep pending.

MAX_NR_CONSECUTIVE_UNCHANGED_PIXELS = 3 # max nr. of consecutive unchanged
                                        # pixels before a new RASET is issued

# Registers        
ST7796S_CSCON   =  0xF0 # Command Set Control
ST7796S_MADCTL  =  0x36 # Memory Data Access Control
ST7796S_COLMOD  =  0x3A # Interface Pixel Format
ST7796S_DIC     =  0xB4 # Display Inversion Control
ST7796S_EM      =  0xB7 # Entry Mode Set
ST7796S_PWR2    =  0xC1 # Power Control 2
ST7796S_PWR3    =  0xC2 # Power Control 3
ST7796S_VCMPCTL =  0xC5 # VCOM Control
ST7796S_VCMOST  =  0xC6 # VCOM Offset Register
ST7796S_DOCA    =  0xE8 # Display Output Ctrl Adjust
ST7796S_PGC     =  0xE0 # Positive Gamma Control
ST7796S_NGC     =  0xE1 # Negative Gamma Control
ST7796S_INVOFF  =  0x20 # Display Inversion Off
ST7796S_INVON   =  0x21 # Display Inversion On
ST7796S_WRCTRLD =  0x53 # Write CTRL Display
ST7796S_DISPOFF =  0x28 # Display Off
ST7796S_DISPON  =  0x29 # Display On
ST7796S_SLPOUT  =  0x11 # Sleep Out
ST7796S_SWRESET =  0x01 # Software reset
ST7796S_RAMWR   =  0x2C # Memory write
ST7796S_RASET   =  0x2B # Raset
ST7796S_CASET   =  0x2A # Caset

#MAX_ROW = 240
#MAX_COLUMN = 320
MAX_ROW = 320
MAX_COLUMN = 480

TEXTGLYPHS = {
    'right_arrow': '\x1a',
    'degrees': '\xf8',
    'left_arrow' : '\x1b'}

class ST7796sParseException(Exception):
    def __init__(self, value):
        super(ST7796sParseException, self).__init__(value)
        self.value = value

def _parse_rect(rect_string):
    components = rect_string.split(',')

    try:
        elements = tuple(int(math.floor(float(p.strip()))) for p in components)
    except ValueError:
        raise ST7796sParseException("Malformed rectangle '%s'"
                                    % (rect_string,))

    if len(elements) != 4:
        raise ST7796sParseException("Malformed rectangle '%s'"
                                    % (rect_string,))
    for i in range(0, len(elements) - 1):
        if (elements[i] < 0 or
           ( (i%1 == 1) and elements[i] > MAX_ROW) or
           ( (i%1 == 0) and elements[i] > MAX_COLUMN)):
            raise ST7796sParseException("Rectangle '%s' outside display"
                                        % (rect_string,))

    return {
        'left': elements[0],
        'top' : elements[1],
        'right' : elements[2],
        'bottom' : elements[3]}

def _parse_color(color_string):
    components = color_string.split(',')

    try:
        elements = tuple(max(0, min(255, math.floor(float(p.strip()))))
                         for p in components)
    except ValueError:
        raise ST7796sParseException("Malformed color '%s'" % (color_string,))

    if len(elements) != 3:
        raise ST7796sParseException("Malformed color '%s'" % (color_string,))

    # convert to RGB565
    _r = int(elements[0])
    _g = int(elements[1])
    _b = int(elements[2])

    color = ((_r // 8) << 11) | ((_g // 4) << 5) | (_b // 8)

    return color

def _parse_glyph(data):
    glyph_data = []
    width = None
    height = 0

    for line in data.split('\n'):
        line = line.strip().replace('.', '0').replace('*', '1')
        if not line:
            continue
        height = height + 1
        if width is None:
            width = len(line)
        elif len(line) != width or line.replace('0', '').replace('1', ''):
            raise ST7796sParseException("Invalid glyph line %d" % (height,))

        linedata = bytearray((width+7) // 8)
        index = 0
        while line:
            this_chunk = line[0:8]
            line = line[8:]

            # pad chunk if less than 8 pixels wide
            chunk_length = len(this_chunk)
            if chunk_length != 8:
                this_chunk += '0' * (8 - chunk_length)

            linedata[index] = int(this_chunk, 2)
            index += 1

        glyph_data.append(linedata)

    return glyph_data, width, height

class ResetHelper(object):
    def __init__(self, disp, resx_pin, csx_pin, rdx_pin, mcu, cmd_queue):
        self.mcu_csx = None
        self.mcu_rdx = None
        self.display = disp

        self.mcu = mcu
        self.cmd_queue = cmd_queue

        self.mcu_resx = bus.MCU_bus_digital_out(
            self.mcu,
            resx_pin,
            self.cmd_queue)

        if csx_pin is not None:
            self.mcu_csx = bus.MCU_bus_digital_out(
                self.mcu,
                csx_pin,
                self.cmd_queue)

        if rdx_pin is not None:
            self.mcu_rdx = bus.MCU_bus_digital_out(
                self.mcu,
                rdx_pin,
                self.cmd_queue)

    def init(self):
        curtime = self.mcu.get_printer().get_reactor().monotonic()

        print_time = self.mcu.estimated_print_time(curtime)
        print_time += ST7796_MINIMAL_DELAY

        minclock = self.mcu.print_time_to_clock(print_time)
        self.mcu_resx.update_digital_out(0, minclock=minclock)

        if self.mcu_rdx is not None or self.mcu_csx is not None:
            print_time = print_time + ST7796_MINIMAL_DELAY
            minclock = self.mcu.print_time_to_clock(print_time)
            if self.mcu_rdx is not None:
                self.mcu_rdx.update_digital_out(1, minclock=minclock)
                self.mcu_csx.update_digital_out(0, minclock=minclock)

        print_time = print_time + ST7796_MINIMAL_DELAY
        minclock = self.mcu.print_time_to_clock(print_time)
        self.mcu_resx.update_digital_out(1, minclock=minclock)

        # Force a delay on the command queue before transmitting
        # the sleep out command
        print_time = print_time + ST7796_RESET_DELAY
        minclock = self.mcu.print_time_to_clock(print_time)
        self.mcu_resx.update_digital_out(1, minclock=minclock)

        self.display.send([[ST7796S_SLPOUT]]) # Sleep out

        # Force a delay on the command queue before transmitting
        # any subsequent commands.
        print_time = print_time + ST7796_MINIMAL_DELAY
        minclock = self.mcu.print_time_to_clock(print_time)
        self.mcu_resx.update_digital_out(1, minclock=minclock)

class PixelRunIterator(object):
    def __init__(self, pixeldata, begin = 0, end = None):
        self.data = pixeldata

        if not end:
            end = 8 * len(pixeldata)
        else:
            end = min(8 * len(pixeldata), end)

        self.next_index = begin // 8
        self.current_bit = 0x80 >> (begin & 7)

        if self.next_index < len(self.data) and begin < end:
            self.current_byte = self.data[self.next_index]
            self.remaining = end - begin
        else:
            self.current_byte = 0
            self.remaining = 0
        self.next_index += 1

    def __iter__(self):
        return self

    def __next__(self):
        if self.remaining == 0:
            raise StopIteration

        thisbit = ((self.current_byte & self.current_bit) != 0)
        full_byte = 0xff if thisbit else 0

        count = 1

        while True:
            self.remaining -= 1
            if self.remaining == 0:
                break

            self.current_bit >>= 1
            if self.current_bit == 0:
                self.current_bit = 0x80
                while True:
                    self.current_byte = self.data[self.next_index]
                    self.next_index += 1
                    if self.current_byte != full_byte or self.remaining < 8:
                        break
                    count += 8
                    self.remaining -= 8
                    if self.remaining == 0:
                        break

            if self.remaining == 0:
                break

            if ((self.current_byte & self.current_bit) != 0) != thisbit:
                break
            count = count + 1

        return thisbit, count
    def next(self): # python 2 compatibility
        return self.__next__()

class PackBitsStream(object):
    LITERAL=-1
    LITERAL_DCX=-2
    def __init__(self, oid, send_cmd):
        self.oid = oid
        self.send_cmd = send_cmd
        self.buffer = bytearray()

    def add_command(self, data):
        self._extend_buffer(PackBitsStream.LITERAL_DCX, data)

    def add_literal_data(self, data):
        data_len = len(data)
        for write_pos in range(0, data_len, 64):
            nr_bytes = min(64, data_len - write_pos)
            self._extend_buffer(PackBitsStream.LITERAL,
                                data[write_pos: write_pos + nr_bytes])

    def add_repeating_data(self, runlength, data):
        while runlength > 0:
            section = min(129, runlength)
            runlength -= section
            self._extend_buffer(section, data)

    def _extend_buffer(self, header, data):
        if not data:
            return
        remaining = MAX_BYTES_IN_REQUEST - len(self.buffer) - 1

        if len(data) > remaining:
            # data will not fit
            if header == PackBitsStream.LITERAL:
                self._extend_buffer(PackBitsStream.LITERAL,
                                    data[:remaining])
                self._extend_buffer(PackBitsStream.LITERAL, data[remaining:])
            elif header == PackBitsStream.LITERAL_DCX:
                # Assert DCX on first write
                self._extend_buffer(PackBitsStream.LITERAL_DCX,
                                    data[:remaining])
                self._extend_buffer(PackBitsStream.LITERAL, data[remaining:])
            else:
                # flush and try again
                self.flush()
                self._extend_buffer(header, data)
        else:
            if header == PackBitsStream.LITERAL:
                self.buffer.append(len(data) + 63)
            elif header == PackBitsStream.LITERAL_DCX:
                self.buffer.append(len(data) - 1)
            else:
                self.buffer.append(header + 126) # repeat
            self.buffer.extend(data)

            if len(self.buffer) >= MAX_BYTES_TO_KEEP_PENDING:
                self.flush()

    def flush(self):
        if self.buffer:
            self.send_cmd.send([self.oid, self.buffer],
                            reqclock=BACKGROUND_PRIORITY_CLOCK)

            self.buffer = bytearray()

class BitmapWriterHelper(object):
    def __init__(self, display, fgcolor, bgcolor, start_x, start_y):
        self.stream = PackBitsStream(display.oid, display.send_cmd)
        self.fgcolor = fgcolor
        self.bgcolor = bgcolor

        self.start_x = start_x
        self.start_y = start_y

        self.literal_buffer = None

        self.last_row = None
        self.last_begin = None
        self.last_end = None

    def flush(self):
        if self.last_row != None:
            # at least one row as been written
            self._flush_literal_buffer()
            self.stream.flush()

    def _flush_literal_buffer(self):
        if not self.literal_buffer:
            return

        self.stream.add_literal_data(self.literal_buffer)

        self.literal_buffer = None

    def write(self, row, data, begin, end):
        if  begin != self.last_begin or end != self.last_end:
            caset_update = True
            raset_update = not self.last_row or self.last_row != row
        else:
            caset_update = False
            raset_update = not self.last_row or (self.last_row+1) != row

        self.last_begin = begin
        self.last_end = end
        self.last_row = row

        if caset_update or raset_update:
            # Finish previous pixel stream
            self._flush_literal_buffer()

        if caset_update:
            x_begin = begin + self.start_x
            x_end = end + self.start_x - 1 # CASET end column is included!

            self.stream.add_command(
              [ST7796S_CASET, # CASET
                (x_begin >> 8) & 0xff,
                x_begin & 0xff,
                (x_end >> 8) & 0xff,
                x_end & 0xff
              ])
        if raset_update:
            y_begin = row + self.start_y
            y_end = MAX_ROW - 1

            self.stream.add_command(
              [ST7796S_RASET, # RASET
                (y_begin >> 8) & 0xff, 
                y_begin & 0xff, 
                (y_end >> 8) & 0xff,
                y_end & 0xff
              ])

        if caset_update or raset_update:
            # start new pixel stream
            self.stream.add_command([ST7796S_RAMWR]) # data, no continuation

        for is_foreground, runlength in PixelRunIterator(data, begin, end):
            value = self.fgcolor if is_foreground else self.bgcolor

            hi = (value >> 8) & 0xff
            lo = value & 0xff

            if runlength < 2:
                if not self.literal_buffer:
                    self.literal_buffer = bytearray([hi, lo])
                else:
                    self.literal_buffer.extend([hi, lo])
            else:
                self._flush_literal_buffer()
                self.stream.add_repeating_data(runlength, [hi, lo])

class Framebuffer(object):
    def __init__(self, display, origin_x, origin_y,
                 size_x, size_y, fgcolor, bgcolor):
        self.size_x = size_x
        self.display = display
        self.size_y = size_y
        self.origin_x = origin_x
        self.end_x = origin_x + size_x
        self.origin_y = origin_y
        self.fgcolor = fgcolor
        self.bgcolor = bgcolor
        self.icons = {}

        self.framebuffer = self.__makebuffer()
        self.old_framebuffer = [None for _i in range(self.size_y)]

        self.nr_columns = size_x // 8
        self.nr_rows = size_y // 16

    def __makebuffer(self):
        # we are working with pixels not columns
        return [bytearray(self.size_x) for _i in range(self.size_y)]
        #return [bytearray((self.size_x+7)//8) for _i in range(self.size_y)]

    def flush(self):
        writer = BitmapWriterHelper(
            self.display, self.fgcolor, self.bgcolor,
            self.origin_x, self.origin_y)
        
        for row in range(self.size_y):
            old_row = self.old_framebuffer[row]
            new_row = self.framebuffer[row]
            for begin, end in Framebuffer.get_pixel_changes(old_row, new_row,
                                                            self.size_x):
                writer.write(row, new_row, begin, end)
            
        writer.flush()
        self.old_framebuffer[:] = self.framebuffer

    def full_flush(self):
        writer = BitmapWriterHelper(
            self.display, self.fgcolor, self.bgcolor,
            self.origin_x, self.origin_y)

        for row in range(self.size_y):
            new_row = self.framebuffer[row]
            writer.write(row, new_row, 0, self.size_x)
            
        writer.flush()
        self.old_framebuffer[:] = self.framebuffer

    @staticmethod
    def get_pixel_changes(old_row, new_row, row_width):
        if old_row == None:
            return [(0, row_width)] # Issue full change

        if old_row == new_row:
            return [] # shortcut for identical data.

        differences = bytearray([ b1 ^ b2 for b1, b2 in zip(old_row, new_row)])
        changes = []

        merge_with_last = False
        current = 0

        for value, runlength in PixelRunIterator(differences, 0, row_width):
            begin = current
            current += runlength
            end = begin + runlength # exclusive

            if  value:
                if merge_with_last:
                    merge_with_last = False
                    begin = changes.pop()[0]

                changes.append((begin, end))
            elif runlength <= MAX_NR_CONSECUTIVE_UNCHANGED_PIXELS and changes:
                merge_with_last = True # Suppress RASET change for this run
        return changes

    def write_glyph(self, _x, _y, glyph_name):
        icon = self.icons.get(glyph_name)
        if icon is not None and _x < self.nr_columns - 1:
            # Draw icon in graphics mode
            for icondata in icon:
                self.write_graphics(_x, _y, icondata)
                _x += 1

            return 2
        char = TEXTGLYPHS.get(glyph_name)
        if char is not None:
            # Draw character
            self.write_text(_x, _y, char)
            return 1
        return 0

    def clear(self):
        self.framebuffer = self.__makebuffer()

    def set_glyphs(self, glyphs):
        for glyph_name, glyph_data in glyphs.items():
            icon = glyph_data.get('icon16x16')
            if icon is not None:
                self.icons[glyph_name] = icon

    def write_text(self, _x, _y, data):
        if _x + len(data) > self.nr_columns:
            data = data[:self.nr_columns - min(_x, self.nr_columns)]

        top_row = 16 * _y

        for char in data:
            font_char = font8x14.VGA_FONT[ord(char)]
            for row in range(14):
                self.framebuffer[top_row + row][_x] = font_char[row]
            _x += 1

    def write_graphics(self, _x, _y, data):
        top_row = _y * 16

        for row, content in enumerate(data):
            self.framebuffer[top_row + row][_x] = content

    def binary_content(self, content):
      b = bytearray()
      for x in content:
        if x:
          b.append("1")
        else:
          b.append("0")
      return b

    def write_framebuffer(self, fb):
        # TODO: Bounds checking
        #for x, y in zip(range(0, 200), range(0, 200)):
        #  self.framebuffer[y][x] = 1
        for row, content in enumerate(fb.framebuffer):
          self.framebuffer[fb.origin_y + row][fb.origin_x:fb.end_x] = content
          #logging.info("content: %s", self.binary_content(content))
          #targetRow = self.framebuffer[srcFramebuffer.origin_y]
          #self.framebuffer[fb.origin_y + row][fb.origin_x:fb.end_x] = [1] * fb.size_x

    def write_bitmap(self, _x, _y, bitmap):
        if not bitmap:
            return

        shift_bits = _x & 7
        carry_bits = 8 - shift_bits

        start_column = _x // 8

        for row, row_data in enumerate(bitmap):
            output_row = self.framebuffer[_y+row]
            carry = output_row[start_column] & (0xff << carry_bits)
            nr_columns = min(len(row_data), self.size_x - start_column)

            for col in range(nr_columns):
                output_row[start_column + col] = (
                    carry | (row_data[col] >> shift_bits))
                carry = (row_data[col] << carry_bits) & 0xff

            if shift_bits > 0:
                # Write out carry, but avoid writing beyond the end of the row.
                end_column = nr_columns + start_column
                if end_column < self.size_x:
                    output_row[end_column] = (
                        (output_row[end_column] & (0xff >> shift_bits)) | carry)

    def get_dimensions(self):
        return (self.nr_columns, self.nr_rows)

class ST7796SButton(object):
    def __init__(self, display, config):
        if config['rect'] is None:
            raise ST7796sParseException("Missing rect")

        if config['fgcolor'] is None:
            raise ST7796sParseException("Missing fgcolor")

        if config['bgcolor'] is None:
            raise ST7796sParseException("Missing bgcolor")

        if config['glyph'] is None:
            raise ST7796sParseException("Missing glyph")

        rect = _parse_rect(config['rect'])
        fgcolor = _parse_color(config['fgcolor'])
        bgcolor = _parse_color(config['bgcolor'])

        self.glyph, self.glyph_width, self.glyph_height = _parse_glyph(config['glyph'])

        self.top = rect['top']
        self.left = rect['left']
        self.width = rect['right'] - self.left
        self.height = rect['bottom'] - self.top

        self._framebuffer = Framebuffer(display,
                                        self.left, self.top,
                                        self.width, self.height,
                                        fgcolor, bgcolor)

        self.glyph_x = (self.width - self.glyph_width) // 2
        self.glyph_y = (self.height - self.glyph_height) // 2

        self._framebuffer.write_bitmap(self.glyph_x, self.glyph_y, self.glyph)

    def flush(self):
        self._framebuffer.flush()

class ST7796S(object):
    def __init__(self, config):
        logging.info("ST7796S __init__")
        printer = config.get_printer()

        try:
            self.fgcolor = _parse_color(config.get("fgcolor", "255,255,255"))
        except ST7796sParseException as err:
            raise config.error(("Error while parsing ST7796s fgcolor: %s")
                               %(err.value,))

        try:
            self.bgcolor = _parse_color(config.get("bgcolor", "0,0,0"))
        except ST7796sParseException as err:
            raise config.error(("Error while parsing ST7796s bgcolor: %s")
                               %(err.value,))

        # pin config
        ppins = printer.lookup_object('pins')

        # mandatory pins
        pins = {name:ppins.lookup_pin(config.get(name + '_pin'))
                for name in ['dcx', 'wrx', 'd8', 'd9', 'd10',
                             'd11', 'd12', 'd13', 'd14', 'd15']}

        mcu = None
        for pin_params in pins.values():
            if mcu is not None and pin_params['chip'] != mcu:
                raise ppins.error("st7796 all pins must be on same mcu")
            mcu = pin_params['chip']
        self.pins = {
            name:pin_params['pin'] for name, pin_params in pins.iteritems()
        }
        self.mcu = mcu
        self.oid = self.mcu.create_oid()
        self.mcu.register_config_callback(self.build_config)
        self.cmd_queue = self.mcu.alloc_command_queue()
        self.send_cmd = None
        self.bitmap_cmd = None
        self.screen_framebuffer = None

        self.reset = ResetHelper(
            self,
            config.get('resx_pin', None),
            config.get('csx_pin', None),
            config.get('rdx_pin', None),
            mcu,
            self.cmd_queue)

        self.screen_width = int(config.get("screen_width", None).strip())
        self.screen_height = int(config.get("screen_height", None).strip())

        global MAX_ROW, MAX_COLUMN
        MAX_ROW = self.screen_height
        MAX_COLUMN = self.screen_width

        self.screen_framebuffer = Framebuffer(self,
                                              0, 0,
                                              self.screen_width,
                                              self.screen_height,
                                              self.fgcolor, self.bgcolor)

        try:
            menu_rect = _parse_rect(config.get('rect'))
        except ST7796sParseException as err:
            raise config.error(("Error while parsing ST7796s rect: %s")
                               %(err.value,))

        self.menu_framebuffer = Framebuffer(self,
                                            menu_rect['left'],
                                            menu_rect['top'],
                                            menu_rect['right'],
                                            menu_rect['bottom'],
                                            self.fgcolor, self.bgcolor)

        # discover soft buttons
        self.__soft_buttons = []

        for i in range(999):
            button_config = {
                'rect' : config.get('button%d_rect' %(i,), None),
                'fgcolor' : config.get('button%d_fgcolor'%(i,), None),
                'bgcolor' : config.get('button%d_bgcolor'%(i,), None),
                'glyph' : config.get('button%d_glyph'%(i,), None)
            }

            if None not in button_config.values():
                try:
                    self.__soft_buttons.append(
                        ST7796SButton(self, button_config))
                except ST7796sParseException as err:
                    raise config.error(
                        "Error while parsing ST7796s button %d: %s"
                        %(i, err.value))
            else:
                break

    def build_config(self):
        # st7796s uses same interface as st7789v
        self.mcu.add_config_cmd(
            "config_ST7788v oid=%u dcx_pin=%s wrx_pin=%s d8_pin=%s"
            " d9_pin=%s d10_pin=%s d11_pin=%s d12_pin=%s d13_pin=%s"
            " d14_pin=%s d15_pin=%s" % (
                self.oid, self.pins['dcx'], self.pins['wrx'], self.pins['d8'],
                self.pins['d9'], self.pins['d10'], self.pins['d11'],
                self.pins['d12'], self.pins['d13'], self.pins['d14'],
                self.pins['d15']
                )
            )

        self.send_cmd = self.mcu.lookup_command(
            "ST7896v_send_cmd oid=%c data=%*s", cq=self.cmd_queue)

    def send(self, cmds):
        stream = PackBitsStream(self.oid, self.send_cmd)

        for cmd in cmds:
            stream.add_command(cmd)

        stream.flush()

    def flush(self):
        self.menu_framebuffer.flush()

    def init(self):
        logging.debug("st7799s init display")

        # Initialisation sequence from marlin
        # Lines idicated with an asterisk (*) indicate where registers are
        
        sw_reset_cmd = [
          [ST7796S_SWRESET]
        ]
        self.send(sw_reset_cmd)
        time.sleep(0.12)
        
        # delay 100ms
        slp_out_cmd = [
          [ST7796S_SLPOUT]
        ]
        self.send(slp_out_cmd)
        time.sleep(0.02)
        # delay 20ms

        self.reset.init()

        # From Marlin
        init_cmds = [
          [ST7796S_CSCON, 0xC3], #enable command 2 part I
          [ST7796S_CSCON, 0x96], #enable command 2 part II
          [ST7796S_MADCTL, 0x20 | 0x00], # Orientation, RGB
          [ST7796S_COLMOD, 0x55],
          [ST7796S_DIC, 0x01], # 1-dot inversion
          [ST7796S_EM, 0xC6],

          [ST7796S_PWR2, 0x15],
          [ST7796S_PWR3, 0xAF],

          [ST7796S_VCMPCTL, 0x22],
          [ST7796S_VCMOST, 0x00],

          [ST7796S_DOCA, 0x40, 0x8A, 0x00, 0x00, 0x29, 0x19, 0xA5, 0x33],

          # Gamma correction
          [ST7796S_PGC, 0xF0, 0x04, 0x08, 0x09, 0x08, 0x15, 0x2F, 0x42, 0x46, 0x28, 0x15, 0x16, 0x29, 0x2D],
          [ST7796S_NGC, 0xF0, 0x04, 0x09, 0x09, 0x08, 0x15, 0x2E, 0x46, 0x46, 0x28, 0x15, 0x15, 0x29, 0x2D],

          # Display inversion
          # [ST7796S_INVON], #inverted
          [ST7796S_INVOFF], #non inverted

          [ST7796S_WRCTRLD, 0x24],
          [ST7796S_CSCON, 0x3C], #disable command 2 part I
          [ST7796S_CSCON, 0x69], #disable command 2 part II 
          # [ST7796S_DISPON] # enable display  
        ]

        self.send(init_cmds)
#                                            menu_rect['left'],
#                                            menu_rect['top'],
#                                            menu_rect['right'],
#                                            menu_rect['bottom'],
        #time.sleep(0.5)
        # Fill screen in black
        intermediate_buffer = Framebuffer(self, 0, 60, 480, 190, 0, 0)
        intermediate_buffer.flush()


        for button in self.__soft_buttons:
            button.flush()

        self.menu_framebuffer.flush()

        self.send([[ST7796S_DISPON]]) # display on
        logging.debug("st7799s display init done!")

    def set_glyphs(self, glyphs):
        self.menu_framebuffer.set_glyphs(glyphs)

    def write_text(self, _x, _y, data):
        self.menu_framebuffer.write_text(_x, _y, data)

    def write_graphics(self, _x, _y, data):
        self.menu_framebuffer.write_graphics(_x, _y, data)

    def write_glyph(self, _x, _y, glyph_name):
        return self.menu_framebuffer.write_glyph(_x, _y, glyph_name)

    def clear(self):
        self.menu_framebuffer.clear()
    
    def get_dimensions(self):
        return self.menu_framebuffer.get_dimensions()
