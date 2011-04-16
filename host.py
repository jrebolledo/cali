#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, os #system managements
import getopt
import wx
import wx.grid
import serial
import glob
from xbee_api import * 
import threading
import datetime
import ast
import math


class regla_manual(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: regla_manual.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.SH  = None
        self.panel_2 = wx.Panel(self, -1)
        self.sizer_24_staticbox = wx.StaticBox(self, -1, "Sincronizar Fecha/Hora")
        self.sizer_25_staticbox = wx.StaticBox(self, -1, "Limpiar EEprom")
        self.sizer_23_staticbox = wx.StaticBox(self, -1, "opciones")
        self.sizer_8_staticbox = wx.StaticBox(self, -1, "Seleccione IOS")
        self.sizer_13_staticbox = wx.StaticBox(self.panel_2, -1, "Hora")
        self.sizer_11_staticbox = wx.StaticBox(self.panel_2, -1, "Comienza")
        self.sizer_15_staticbox = wx.StaticBox(self.panel_2, -1, "Hora")
        self.sizer_12_staticbox = wx.StaticBox(self.panel_2, -1, "Termina")
        self.sizer_10_staticbox = wx.StaticBox(self.panel_2, -1, u"Periodo de Aplicación")
        self.sizer_16_staticbox = wx.StaticBox(self, -1, "Accion")
        self.sizer_21_staticbox = wx.StaticBox(self, -1, u"Conexión")
        self.dev_selection = wx.ComboBox(self, -1, choices=["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"], style=wx.CB_DROPDOWN)
        self.conectar = wx.Button(self, -1, "conectar")
        self.label_1 = wx.StaticText(self, -1, "")
        self.sync_datetime = wx.Button(self, -1, "Sincronizar")
        self.clean_eeprom = wx.Button(self, -1, "Limpiar")
        self.IO_0 = wx.CheckBox(self, -1, "0")
        self.IO_1 = wx.CheckBox(self, -1, "1")
        self.IO_2 = wx.CheckBox(self, -1, "2")
        self.IO_3 = wx.CheckBox(self, -1, "3")
        self.IO_4 = wx.CheckBox(self, -1, "4")
        self.IO_5 = wx.CheckBox(self, -1, "5")
        self.start_date = wx.DatePickerCtrl(self.panel_2, -1)
        self.start_hour = wx.TextCtrl(self.panel_2, -1, "")
        self.start_min = wx.TextCtrl(self.panel_2, -1, "")
        self.end_date = wx.DatePickerCtrl(self.panel_2, -1)
        self.end_hour = wx.TextCtrl(self.panel_2, -1, "")
        self.end_min = wx.TextCtrl(self.panel_2, -1, "")
        self.radio_encender = wx.RadioButton(self, -1, "Encender")
        self.radio_apagar = wx.RadioButton(self, -1, "Apagar")
        self.send_manual = wx.Button(self, -1, "Enviar Regla Manual")

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_BUTTON, self.sinc, self.sync_datetime)
        self.Bind(wx.EVT_BUTTON, self.clean_eeprom_comm, self.clean_eeprom)
        self.Bind(wx.EVT_BUTTON, self.send_regla, self.send_manual)
        self.Bind(wx.EVT_BUTTON, self.conectar_serial, self.conectar)
        # end wxGlade


    def __set_properties(self):
        # begin wxGlade: manual.__set_properties
        self.SetTitle("Regla Manual")
        now = datetime.datetime.now()
        self.dev_selection.SetMinSize((100, 25))
        self.start_hour.SetMinSize((40, 25))
        self.start_hour.SetValue(str(now.hour))
        self.start_min.SetMinSize((40, 25))
        self.start_min.SetValue(str(now.minute))
        self.end_hour.SetMinSize((40, 25))
        self.end_hour.SetValue(str(now.hour))
        self.end_min.SetMinSize((40, 25))
        self.end_min.SetValue(str(now.minute))
        # end wxGlade

    def __do_layout(self):
# begin wxGlade: regla_manual.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_16 = wx.StaticBoxSizer(self.sizer_16_staticbox, wx.VERTICAL)
        sizer_17 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_10 = wx.StaticBoxSizer(self.sizer_10_staticbox, wx.HORIZONTAL)
        sizer_12 = wx.StaticBoxSizer(self.sizer_12_staticbox, wx.VERTICAL)
        sizer_15 = wx.StaticBoxSizer(self.sizer_15_staticbox, wx.HORIZONTAL)
        sizer_11 = wx.StaticBoxSizer(self.sizer_11_staticbox, wx.VERTICAL)
        sizer_13 = wx.StaticBoxSizer(self.sizer_13_staticbox, wx.HORIZONTAL)
        sizer_8 = wx.StaticBoxSizer(self.sizer_8_staticbox, wx.VERTICAL)
        sizer_9 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_23 = wx.StaticBoxSizer(self.sizer_23_staticbox, wx.HORIZONTAL)
        sizer_25 = wx.StaticBoxSizer(self.sizer_25_staticbox, wx.HORIZONTAL)
        sizer_24 = wx.StaticBoxSizer(self.sizer_24_staticbox, wx.HORIZONTAL)
        sizer_21 = wx.StaticBoxSizer(self.sizer_21_staticbox, wx.HORIZONTAL)
        sizer_22 = wx.BoxSizer(wx.VERTICAL)
        sizer_21.Add(self.dev_selection, 0, 0, 0)
        sizer_22.Add(self.conectar, 0, 0, 0)
        sizer_22.Add(self.label_1, 0, 0, 0)
        sizer_21.Add(sizer_22, 1, wx.EXPAND, 0)
        sizer_1.Add(sizer_21, 1, wx.EXPAND, 0)
        sizer_24.Add(self.sync_datetime, 0, 0, 0)
        sizer_23.Add(sizer_24, 1, wx.EXPAND, 0)
        sizer_25.Add(self.clean_eeprom, 0, 0, 0)
        sizer_23.Add(sizer_25, 1, wx.EXPAND, 0)
        sizer_1.Add(sizer_23, 1, wx.EXPAND, 0)
        sizer_9.Add(self.IO_0, 0, 0, 0)
        sizer_9.Add(self.IO_1, 0, 0, 0)
        sizer_9.Add(self.IO_2, 0, 0, 0)
        sizer_9.Add(self.IO_3, 0, 0, 0)
        sizer_9.Add(self.IO_4, 0, 0, 0)
        sizer_9.Add(self.IO_5, 0, 0, 0)
        sizer_8.Add(sizer_9, 1, 0, 0)
        sizer_1.Add(sizer_8, 1, wx.EXPAND, 0)
        sizer_11.Add(self.start_date, 0, 0, 0)
        sizer_13.Add(self.start_hour, 0, 0, 0)
        sizer_13.Add(self.start_min, 0, 0, 0)
        sizer_11.Add(sizer_13, 1, wx.EXPAND, 0)
        sizer_10.Add(sizer_11, 1, wx.SHAPED, 0)
        sizer_12.Add(self.end_date, 0, 0, 0)
        sizer_15.Add(self.end_hour, 0, 0, 0)
        sizer_15.Add(self.end_min, 0, 0, 0)
        sizer_12.Add(sizer_15, 1, wx.EXPAND, 0)
        sizer_10.Add(sizer_12, 1, wx.SHAPED, 0)
        self.panel_2.SetSizer(sizer_10)
        sizer_1.Add(self.panel_2, 1, 0, 0)
        sizer_17.Add(self.radio_encender, 0, 0, 0)
        sizer_17.Add(self.radio_apagar, 0, 0, 0)
        sizer_16.Add(sizer_17, 1, wx.EXPAND, 0)
        sizer_16.Add(self.send_manual, 0, 0, 0)
        sizer_1.Add(sizer_16, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_1)
        sizer_1.Fit(self)
        self.Layout()
        # end wxGlade
        # end wxGlade
    

        
    
    def writePkg(self,pkg):
        
        self.SH.bufferIn = {} #clean bufferIn
        self.SH.packAndSend(pkg)
        TIMEOUT = False
        startTime = time.time()
        TIMEOUT_SET = 1
        while not (TIMEOUT or (self.SH.bufferIn.has_key('method'))):
            TIMEOUT = False
            if (time.time() - startTime) > TIMEOUT_SET:
                TIMEOUT = True
        
        #timeout handling
        if TIMEOUT:
            #print "Error Timeout Writing"
            return False
        else:
            if self.SH.bufferIn['method'] == pkg['method'] and self.SH.bufferIn['params'][0] == 0:
                return True
            else:
                return False
    
    def conectar_serial(self,event):
        dev = self.dev_selection.GetValue()
        try:
            self.SH   =   UARTApi(dev=dev,baud=9600)
            print 'Conectado al puerto %s Correctamente'%(dev)
            self.label_1.SetLabel('Conectado')
        except:
            print 'error al conectar intente otro puerto'
            self.label_1.SetLabel('Error, elija otro')
    
    def clean_eeprom_comm(self, event): # wxGlade: regla_manual.<event_handler>
        print "clean_eeprom_comm"
        pkg = {'method':28,'params':[0],'id':2}
        print pkg
        self.writePkg(pkg)
        
    def sinc(self,event):
        now = datetime.datetime.now()
        pkg = {'method':27,'params':[now.year-2000,now.month,now.day,now.hour,now.minute],'id':2}
        print pkg
        self.writePkg(pkg)
        
    def send_regla(self, event): # wxGlade: manual.<event_handler>
        
        #IOindexs = control_data['tags'].keys()
        numIO = 0
        IO_list = []
        for y in range(6):
            a = getattr(self,'IO_%s'%y)
            if a.GetValue():
                numIO = numIO + 1
                IO_list.append(y)
            
        
        from_date = self.start_date.GetValue()
        from_str = [from_date.Year-2000,from_date.Month+1,from_date.Day,int(self.start_hour.GetValue()),int(self.start_min.GetValue())]
        
        end_date = self.end_date.GetValue()
        to_str = [end_date.Year-2000,end_date.Month+1,end_date.Day, int(self.end_hour.GetValue()),int(self.end_min.GetValue())]
        
        forcing_state_to = self.radio_encender.GetValue()
        state = {True:1,False:0}[forcing_state_to]
        
        params = [numIO]+IO_list+from_str+to_str+[state]
        print 'packet: %s'%params
        pkg = {'method':23,'params':params,'id':2}
        self.writePkg(pkg)

# end of class manual


if __name__ == "__main__":
    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    frame_1 = regla_manual(None, -1, "")
    app.SetTopWindow(frame_1)
    frame_1.Show()
    app.MainLoop()
    
    
    