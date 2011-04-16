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

correctionPARAMS = {'VOLT_CC':1,'AMP_CC':1,'PWR_CC':1,'ENR_CC':1}

dict_cali = {'VOLT_CC':{'address':[0,0x14],'len':2, 'shift':0},
             'AMP_CC':{'address':[0,0x16],'len':2, 'shift':0},
             'PWR_CC':{'address':[0,0x18],'len':2, 'shift':0},
             'ENR_CC':{'address':[0,0x1A],'len':2, 'shift':0},
             'A.VRMS':{'address':[0x1,0xC8],'len':4,'shift':0},
             'B.VRMS':{'address':[0x2,0xB4],'len':4,'shift':0},
             'C.VRMS':{'address':[0x3,0xA0],'len':4,'shift':0},
             'A.VGAIN':{'address':[0x1,0x32],'len':2,'shift':0},
             'B.VGAIN':{'address':[0x2,0x1E],'len':2,'shift':0},
             'C.VGAIN':{'address':[0x3,0x0A],'len':2,'shift':0},
             'A.IRMS':{'address':[0x1,0xCC],'len':4,'shift':0},
             'B.IRMS':{'address':[0x2,0xB8],'len':4,'shift':0},
             'C.IRMS':{'address':[0x3,0xA4],'len':4,'shift':0},
             'A.IGAIN':{'address':[0x1,0x30],'len':2,'shift':0},
             'B.IGAIN':{'address':[0x2,0x1C],'len':2,'shift':0},
             'C.IGAIN':{'address':[0x3,0x08],'len':2,'shift':0},
             
             
             'I1THR':{'address':[0x0,0x5C],'len':2,'shift':0},
             'I2THR':{'address':[0x0,0x5E],'len':2,'shift':0},
             
             'A.PA0':{'address':[0x1,0x3E],'len':2,'shift':0},
             'B.PA0':{'address':[0x2,0x2A],'len':2,'shift':0},
             'C.PA0':{'address':[0x3,0x16],'len':2,'shift':0},
             
             'A.PA1':{'address':[0x1,0x40],'len':2,'shift':0},
             'B.PA1':{'address':[0x2,0x2C],'len':2,'shift':0},
             'C.PA1':{'address':[0x3,0x18],'len':2,'shift':0},
             
             'A.PA2':{'address':[0x1,0x42],'len':2,'shift':0},
             'B.PA2':{'address':[0x2,0x2E],'len':2,'shift':0},
             'C.PA2':{'address':[0x3,0x1A],'len':2,'shift':0},
             
             'A.ACT':{'address':[0x1,0xD0],'len':4,'shift':0},
             'B.ACT':{'address':[0x2,0xBC],'len':4,'shift':0},
             'C.ACT':{'address':[0x3,0xA8],'len':4,'shift':0},
             
             'A.REA':{'address':[0x1,0xD4],'len':4,'shift':0},
             'B.REA':{'address':[0x2,0xC0],'len':4,'shift':0},
             'C.REA':{'address':[0x3,0xAC],'len':4,'shift':0},
             
             'PWRP.A':{'address':[0x8,0x01],'len':8,'shift':2},
             'PWRP.B':{'address':[0x8,0x02],'len':8,'shift':2},
             'PWRP.C':{'address':[0x8,0x04],'len':8,'shift':2},
             
             'PWRQ.A':{'address':[0x8,0x11],'len':8,'shift':2},
             'PWRQ.B':{'address':[0x8,0x12],'len':8,'shift':2},
             'PWRQ.C':{'address':[0x8,0x14],'len':8,'shift':2},
             
             'V.A':{'address':[0x8,0x31],'len':8,'shift':2},
             'V.B':{'address':[0x8,0x32],'len':8,'shift':2},
             'V.C':{'address':[0x8,0x34],'len':8,'shift':2},
             
             'I.A':{'address':[0x8,0x41],'len':8,'shift':2},
             'I.B':{'address':[0x8,0x42],'len':8,'shift':2},
             'I.C':{'address':[0x8,0x44],'len':8,'shift':2},
             
             'PF.A':{'address':[0x1,0xC6],'len':2,'shift':0},
             'PF.B':{'address':[0x2,0xB2],'len':2,'shift':0},
             'PF.C':{'address':[0x3,0x9E],'len':2,'shift':0},
            
             'ENRP.A':{'address':[0x8,0xC1],'len':8,'shift':3},
             'ENRP.B':{'address':[0x8,0xC2],'len':8,'shift':3},
             'ENRP.C':{'address':[0x8,0xC4],'len':8,'shift':3}
             
}

def writePkg(pkg):
    
    SH.UART.bufferIn = {} #clean bufferIn
    SH.UART.packAndSend(pkg)
    TIMEOUT = False
    startTime = time.time()
    TIMEOUT_SET = 1
    while not (TIMEOUT or (SH.UART.bufferIn.has_key('method'))):
        TIMEOUT = False
        if (time.time() - startTime) > TIMEOUT_SET:
            TIMEOUT = True
    
    #timeout handling
    if TIMEOUT:
        #print "Error Timeout Writing"
        return False
    else:
        if SH.UART.bufferIn['method'] == pkg['method'] and SH.UART.bufferIn['params'][0] == 0:
            return True
        else:
            return False
        
def convertToInt(array):
    from struct import unpack
    return unpack('>1%s'%{2:'H',4:'L'}[len(array)],''.join([chr(i) for i in array]))[0]

def VirtualToStr(token,cc_correction):
    from struct import unpack
    if token:
        array = SH.UART.bufferIn['params'][6:]
        return float(unpack('>1l',''.join([chr(i) for i in array]))[0]/10.0)*cc_correction
    else:
        return 'NA'

def BytestoStr(token,bytes_len,scale=1,cc_correction=1):
    from struct import unpack
    if token:
        if bytes_len == 8:
            array = SH.UART.bufferIn['params'][6:]
            return float(unpack('>1l',''.join([chr(i) for i in array]))[0] * scale) * cc_correction
        if bytes_len == 2:
            array = SH.UART.bufferIn['params'][2:]
            return float(unpack('>1h',''.join([chr(i) for i in array]))[0] * scale * 1.0)
            
            
def writeandcheckMAXQ(key,value_array=[],verbose=True):
    tries = 0
    while True:
        writing =  writeMAXQ(key,value_array,verbose=verbose)
        if writing:
            return True
    
        tries = tries + 1
        #print 'intento %s'%tries
        if tries == 3:
            return False
        
    return False
        
    
def writeMAXQ(key,value_array=[],verbose=False):
    
    if key == 'start':
        METHOD = 230
        params = [1]
        packet_id = 88
    
    if key == 'getAccum': # attach var def
        METHOD = 231
        params = [0]
        packet_id = 76
        
    if key == 'CleanAccum': # attach var def
        METHOD = 232
        params = [0]
        packet_id = 77
        
    if key == 'endcali':
        METHOD = 230
        params = [0]
        packet_id = 88    
    
    if key == 'CleanUnsyncData':
        METHOD = 232
        params = [1]
        packet_id = 88
        
    if not key in ['getAccum','endcali','CleanUnsyncData','encali','CleanAccum','start']: 
        METHOD = 95
        params = dict_cali[key]['address']+ [dict_cali[key]['len'],dict_cali[key]['shift']] + value_array 
        packet_id = 88
    
    pkg = {'method':METHOD,'params':params,'id':packet_id}
    if verbose:
        pass
        #print 'Enviando\n\tmethod:%s\n\tparams:%s\n\tid:%s'%(pkg['method'],[hex(i) for i in pkg['params']],pkg['id'])
    
    SH.UART.bufferIn = {} #clean bufferIn
    SH.UART.packAndSend(pkg)
    TIMEOUT = False
    startTime = time.time()
    TIMEOUT_SET = 1
    while not (TIMEOUT or (SH.UART.bufferIn.has_key('method'))):
        TIMEOUT = False
        if (time.time() - startTime) > TIMEOUT_SET:
            TIMEOUT = True
    
    #timeout handling
    if TIMEOUT:
        #print "Error Timeout Writing"
        return False
    else:
        if SH.UART.bufferIn['method'] == pkg['method'] and SH.UART.bufferIn['params'][0] == 0:
            return True
        else:
            return False
    
    

def readMAXQ(key,verbose=True):
    METHOD = 98
    params = dict_cali[key]['address'] + [dict_cali[key]['len'],dict_cali[key]['shift']]
    packet_id = 88
    pkg = {'method':METHOD,'params':params,'id':packet_id}
    if verbose:
        pass
        #print 'Enviando\n\tmethod:%s\n\tparams:%s\n\tid:%s'%(pkg['method'],[hex(i) for i in pkg['params']],pkg['id'])
    tries = 0
    while tries<4:
        SH.UART.bufferIn = {} #clean bufferIn
        SH.UART.packAndSend(pkg)
        TIMEOUT = False
        startTime = time.time()
        TIMEOUT_SET = 2
        
        while not (TIMEOUT or (SH.UART.bufferIn.has_key('method'))):
            TIMEOUT = False
            if (time.time() - startTime) > TIMEOUT_SET:
                TIMEOUT = True
        
        if not TIMEOUT:
            return True
        #timeout handling
        #print "intento %s"%tries
        tries = tries +1
    
    return False

class Status_bar(wx.Frame):
    def __init__(self, parent, ID, title, status_h):
        wx.Frame.__init__(self, parent, ID, title)

        self.count = 0
        self.method = status_h
        
        panel = wx.Panel(self, -1)
        vbox = wx.BoxSizer(wx.VERTICAL)
        hbox1 = wx.BoxSizer(wx.HORIZONTAL)
        hbox2 = wx.BoxSizer(wx.HORIZONTAL)
        hbox3 = wx.BoxSizer(wx.HORIZONTAL)

        self.gauge = wx.Gauge(panel, -1, 50, size=(250, 25))
        self.btn1 = wx.Button(panel, wx.ID_OK)
        self.btn2 = wx.Button(panel, wx.ID_STOP)
        self.text = wx.StaticText(panel, -1, "Task to be done")

        hbox1.Add(self.gauge, 1, wx.ALIGN_CENTRE)
        hbox2.Add(self.btn1, 1, wx.RIGHT, 10)
        hbox2.Add(self.btn2, 1)
        hbox3.Add(self.text, 1)
        vbox.Add((0, 50), 0)
        vbox.Add(hbox1, 0, wx.ALIGN_CENTRE)
        vbox.Add((0, 30), 0)
        vbox.Add(hbox2, 1, wx.ALIGN_CENTRE)
        vbox.Add(hbox3, 1, wx.ALIGN_CENTRE)

        panel.SetSizer(vbox)
        self.Centre()

        self.text.SetLabel("En progreso")
        self.method.start()
        
    def OnStop(self):
        self.kill_background_thread = True
        self.text.SetLabel("Task Interrupted")


class SerialHandling():
    def __init__(self,baud=9600,verbose=False):
        CONNECTION_SERIAL_PORT_STATUS   = False
        self.serial_port = ""
        self.baud = baud
        self.verbose = verbose
        self.UART = None

    def connect(self):    
        self.UART   =   UARTApi(dev=self.serial_port,baud=self.baud)
        CONNECTION_SERIAL_PORT_STATUS   = True
        print 'Conectado al puerto %s Correctamente'%(self.serial_port)
        


class SerialSelect(wx.Dialog):
    def __init__(self, *args, **kwds):
        # begin wxGlade: SerialSelect.__init__
        kwds["style"] = wx.DEFAULT_DIALOG_STYLE
        wx.Dialog.__init__(self, *args, **kwds)
        self.label_serialselect = wx.StaticText(self, -1, "Seleccione el dispositivo:\nEj: 13 para COM14 o /dev/ttyUSB0")
        self.text_ctrl_serialselect = wx.TextCtrl(self, -1, "/dev/ttyUSB3")
        self.button_connect = wx.Button(self, -1, "Calibrar")

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_BUTTON, self.conectar, self.button_connect)
        # end wxGlade

    def __set_properties(self):
        # begin wxGlade: SerialSelect.__set_properties
        self.SetTitle("Seleccion de Dispositivo")
        self.SetSize((320, 120))
        self.label_serialselect.SetMinSize((221, 34))
        self.text_ctrl_serialselect.SetMinSize((180, 30))
        self.text_ctrl_serialselect.SetFocus()
        self.button_connect.SetMinSize((85, 30))
        self.button_connect.SetFocus()
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: SerialSelect.__do_layout
        grid_sizer_3 = wx.FlexGridSizer(4, 3, 0, 0)
        grid_sizer_3.Add((20, 20), 0, 0, 0)
        grid_sizer_3.Add((180, 20), 0, 0, 0)
        grid_sizer_3.Add((85, 20), 0, 0, 0)
        grid_sizer_3.Add((20, 30), 0, 0, 0)
        grid_sizer_3.Add(self.label_serialselect, 0, 0, 0)
        grid_sizer_3.Add((85, 30), 0, 0, 0)
        grid_sizer_3.Add((20, 30), 0, 0, 0)
        grid_sizer_3.Add(self.text_ctrl_serialselect, 0, 0, 0)
        grid_sizer_3.Add((85, 30), 0, 0, 0)
        grid_sizer_3.Add((20, 30), 0, 0, 0)
        grid_sizer_3.Add((180, 30), 0, 0, 0)
        grid_sizer_3.Add(self.button_connect, 0, 0, 0)
        self.SetSizer(grid_sizer_3)
        self.Layout()
        # end wxGlade

    def conectar(self, event): # wxGlade: SerialSelect.<event_handler>
        #frame_1.ser.port = self.text_ctrl_serialselect.GetValue()
        
        SH.serial_port = self.text_ctrl_serialselect.GetValue()
        SH.connect()
        
        configframe = ConfigFrame(None, -1, "")
        mediframe = MediFrame(None, -1,"")
        
        configframe.nodos = ''
        mediframe.nodos = ''
        
        mediframe.Show()
        configframe.Show()
        
        writeMAXQ('start');
        writeMAXQ('getAccum');
        
        writeMAXQ('CleanUnsyncData');
        
        self.Hide()
        event.Skip()

class MediFrame(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: MediFrame.__init__
        # begin wxGlade: MyFrame.__init__
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.sizer_1_staticbox = wx.StaticBox(self, -1, "Mediciones Registros Virtuales")
        self.control = wx.Button(self, -1, "Start")
        self.mediciones = wx.grid.Grid(self, -1, size=(1, 1))

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_BUTTON, self.medir, self.control)
        # end wxGlade
        
        global MedBreak

        # end wxGlade

    def __set_properties(self):
        # begin wxGlade: MediFrame.__set_properties
        self.SetTitle("Mediciones")
        self.SetSize((450, 312))
        self.mediciones.CreateGrid(6, 4)
        self.mediciones.SetRowLabelSize(30)
        self.mediciones.SetColLabelValue(0, "Med")
        self.mediciones.SetColLabelValue(1, "A")
        self.mediciones.SetColLabelValue(2, "B")
        self.mediciones.SetColLabelValue(3, "C")
        
        self.mediciones.SetCellValue(0,0,"VRMS(V)")
        self.mediciones.SetCellValue(1,0,"IRMS(A)")
        self.mediciones.SetCellValue(2,0,"PWRP(W)")
        self.mediciones.SetCellValue(3,0,"PWRQ(VAR)")
        self.mediciones.SetCellValue(4,0,"FP")
        self.mediciones.SetCellValue(5,0,"ENRP(Wh)")
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: MediFrame.__do_layout
        sizer_1 = wx.StaticBoxSizer(self.sizer_1_staticbox, wx.VERTICAL)
        sizer_1.Add(self.control, 0, wx.ALIGN_CENTER_HORIZONTAL, 1)
        sizer_1.Add(self.mediciones, 1, wx.EXPAND|wx.ALIGN_CENTER_HORIZONTAL, 6)
        self.SetSizer(sizer_1)
        self.Layout()
        self.Centre()
        # end wxGlade
        
    def medir(self, event): # wxGlade: MediFrame.<event_handler>
        global MedBreak
        if self.control.GetLabelText() == "Start":
            print "START"
            self.control.SetLabel("Stop")
            MedBreak = False
            t_Var = threading.Thread(target = self.medir_var,args=[event])
            t_Var.start()
        else:
            MedBreak = True
            print "STOP"
            self.control.SetLabel("Start")
            
    def medir_var(self,event):
        while not MedBreak:
            
            A_VRMS = VirtualToStr(readMAXQ('V.A'),correctionPARAMS['VOLT_CC'])
            self.mediciones.SetCellValue(0,1,str(A_VRMS))

            B_VRMS = VirtualToStr(readMAXQ('V.B'),correctionPARAMS['VOLT_CC'])
            self.mediciones.SetCellValue(0,2,str(B_VRMS),)
            C_VRMS = VirtualToStr(readMAXQ('V.C'),correctionPARAMS['VOLT_CC'])
            self.mediciones.SetCellValue(0,3,str(C_VRMS))            

            
            A_IRMS = VirtualToStr(readMAXQ('I.A'),correctionPARAMS['AMP_CC'])
            self.mediciones.SetCellValue(1,1,str(A_IRMS))
            B_IRMS = VirtualToStr(readMAXQ('I.B'),correctionPARAMS['AMP_CC'])
            self.mediciones.SetCellValue(1,2,str(B_IRMS))
            C_IRMS = VirtualToStr(readMAXQ('I.C'),correctionPARAMS['AMP_CC'])
            self.mediciones.SetCellValue(1,3,str(C_IRMS))
            
            A_ACT = VirtualToStr(readMAXQ('PWRP.A'),correctionPARAMS['PWR_CC'])
            self.mediciones.SetCellValue(2,1,str(A_ACT))
            B_ACT = VirtualToStr(readMAXQ('PWRP.B'),correctionPARAMS['PWR_CC'])
            self.mediciones.SetCellValue(2,2,str(B_ACT))
            C_ACT = VirtualToStr(readMAXQ('PWRP.C'),correctionPARAMS['PWR_CC'])
            self.mediciones.SetCellValue(2,3,str(C_ACT))
            
            A_REA = VirtualToStr(readMAXQ('PWRQ.A'),correctionPARAMS['PWR_CC'])
            self.mediciones.SetCellValue(3,1,str(A_REA))
            B_REA = VirtualToStr(readMAXQ('PWRQ.B'),correctionPARAMS['PWR_CC'])
            self.mediciones.SetCellValue(3,2,str(B_REA))
            C_REA = VirtualToStr(readMAXQ('PWRQ.C'),correctionPARAMS['PWR_CC'])
            self.mediciones.SetCellValue(3,3,str(C_REA))
            
            A_PF = BytestoStr(readMAXQ('PF.A'),2,scale=pow(2,-14))
            self.mediciones.SetCellValue(4,1,'%3.4g'%A_PF)
            B_PF = BytestoStr(readMAXQ('PF.B'),2,scale=pow(2,-14))
            self.mediciones.SetCellValue(4,2,'%3.4g'%B_PF)
            C_PF = BytestoStr(readMAXQ('PF.C'),2,scale=pow(2,-14))
            self.mediciones.SetCellValue(4,3,'%3.4g'%C_PF)

            ENRP_A = BytestoStr(readMAXQ('ENRP.A'),8,0.1,correctionPARAMS['ENR_CC'])
            self.mediciones.SetCellValue(5,1,'%6.6g'%ENRP_A)

            ENRP_B = BytestoStr(readMAXQ('ENRP.B'),8,0.1,correctionPARAMS['ENR_CC'])
            self.mediciones.SetCellValue(5,2,'%6.6g'%ENRP_B)
            
            ENRP_C = BytestoStr(readMAXQ('ENRP.C'),8,0.1,correctionPARAMS['ENR_CC'])
            self.mediciones.SetCellValue(5,3,'%6.6g'%ENRP_C)

            event.Skip()
            if MedBreak:
                break
        print "QUIT THREAD"
        self.control.SetLabel("Start")

        #self.button_medir.SetLabel("Start")
            
        
        #self.button_medir.SetLabel("Start")  
# end of class MediFrame

class ConfigFrame(wx.Frame):
    def __init__(self, *args, **kwds):
        # begin wxGlade: ConfigFrame.__init__
        self.nodos = {"dir": "0012", "slot": "2"}
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.status_bar = None
        self.kill_backgroud_task = False
        self.notebook = wx.Notebook(self, -1, style=0)
        self.notebook_fase = wx.Panel(self.notebook, -1)
        self.notebook_programar = wx.Panel(self.notebook, -1)
        self.notebook_corriente = wx.Panel(self.notebook, -1)
        self.notebook_volt = wx.Panel(self.notebook, -1)
        self.notebook_registros = wx.Panel(self.notebook, -1)
        self.button_reg_back = wx.Button(self.notebook_registros, -1, "Back")
        self.label_reg_volt = wx.StaticText(self.notebook_registros, -1, u"Razón de Voltajes")
        self.text_ctrl_reg_volt = wx.TextCtrl(self.notebook_registros, -1, "541")
        self.label_V_V = wx.StaticText(self.notebook_registros, -1, "V/V")
        self.label_reg_corriente = wx.StaticText(self.notebook_registros, -1, u"Razón de Corrientes")
        self.text_ctrl_reg_corriente = wx.TextCtrl(self.notebook_registros, -1, "3300")
        self.label_I_I = wx.StaticText(self.notebook_registros, -1, "I/I")
        self.button_reg_calibrar = wx.Button(self.notebook_registros, -1, "Calibrar Registros CC")
        self.combo_box_V_fase = wx.ComboBox(self.notebook_volt, -1, choices=["Fase A", "Fase B", "Fase C"], style=wx.CB_DROPDOWN)
        self.button_V_back = wx.Button(self.notebook_volt, -1, "Back")
        self.label_volt_med = wx.StaticText(self.notebook_volt, -1, "Voltaje Medido")
        self.text_ctrl_volt_med = wx.TextCtrl(self.notebook_volt, -1, "220")
        self.label_V = wx.StaticText(self.notebook_volt, -1, "V")
        self.label_V_tol = wx.StaticText(self.notebook_volt, -1, "Tolerancia")
        self.text_ctrl_V_tol = wx.TextCtrl(self.notebook_volt, -1, "0.5")
        self.label_V_percent = wx.StaticText(self.notebook_volt, -1, "%")
        self.label_V_intent = wx.StaticText(self.notebook_volt, -1, "Intentos")
        self.text_ctrl_V_intent = wx.TextCtrl(self.notebook_volt, -1, "5")
        self.button_V_calibrar = wx.Button(self.notebook_volt, -1, "Calibrar")
        self.grid_voltaje = wx.grid.Grid(self.notebook_volt, -1, size=(1, 1))
        self.combo_box_I_fase = wx.ComboBox(self.notebook_corriente, -1, choices=["Fase A", "Fase B", "Fase C"], style=wx.CB_DROPDOWN)
        self.button_I_back = wx.Button(self.notebook_corriente, -1, "Back")
        self.label_I_med = wx.StaticText(self.notebook_corriente, -1, "Corriente Medida ")
        self.text_ctrl_I_med = wx.TextCtrl(self.notebook_corriente, -1, "25.2")
        self.label_I = wx.StaticText(self.notebook_corriente, -1, "A")
        self.label_I_tol = wx.StaticText(self.notebook_corriente, -1, "Tolerancia")
        self.text_ctrl_I_tol = wx.TextCtrl(self.notebook_corriente, -1, "0.5")
        self.label_I_percent = wx.StaticText(self.notebook_corriente, -1, "%")
        self.label_I_intent = wx.StaticText(self.notebook_corriente, -1, "Intentos")
        self.text_ctrl_I_intent = wx.TextCtrl(self.notebook_corriente, -1, "5")
        self.button_I_calibrar = wx.Button(self.notebook_corriente, -1, "Calibrar")
        self.grid_corriente = wx.grid.Grid(self.notebook_corriente, -1, size=(1, 1))
        self.button_prog_back = wx.Button(self.notebook_programar, -1, "Back")
        self.button_importar = wx.Button(self.notebook_programar, -1, "Importar")
        self.text_ctrl_imp_path = wx.TextCtrl(self.notebook_programar, -1, "")
        self.button_imp_search = wx.Button(self.notebook_programar, -1, "...")
        self.grid_respaldo = wx.grid.Grid(self.notebook_programar, -1, size=(1, 1))
        self.button_exportar = wx.Button(self.notebook_programar, -1, "Exportar")
        self.text_ctrl_exp_path = wx.TextCtrl(self.notebook_programar, -1, "")
        self.button_exp_search = wx.Button(self.notebook_programar, -1, "...")
        self.button_prog_calibrar = wx.Button(self.notebook_programar, -1, "Calibrar")
        self.button_fase_back = wx.Button(self.notebook_fase, -1, "Back")
        self.label_fase_ang = wx.StaticText(self.notebook_fase, -1, u"Ángulo")
        self.text_ctrl_fase_ang = wx.TextCtrl(self.notebook_fase, -1, "0")
        self.label_fase_grados = wx.StaticText(self.notebook_fase, -1, u"°")
        self.label_fase_carga = wx.StaticText(self.notebook_fase, -1, "Tipo de Carga")
        self.radio_box_fase_carga = wx.RadioBox(self.notebook_fase, -1, "", choices=["Capacitiva", "Inductiva"], majorDimension=0, style=wx.RA_SPECIFY_ROWS)
        self.button_fase_calibrar = wx.Button(self.notebook_fase, -1, "Calibrar")
        self.grid_fase = wx.grid.Grid(self.notebook_fase, -1, size=(1, 1))
        self.notebook_pane_ajustes = wx.Panel(self.notebook, -1)

        self.__set_properties()
        self.__do_layout()

        self.Bind(wx.EVT_BUTTON, self.reg_calibrar_thread, self.button_reg_calibrar)
        self.Bind(wx.EVT_BUTTON, self.go_back, self.button_reg_back)
        self.Bind(wx.EVT_BUTTON, self.go_back, self.button_V_back)
        self.Bind(wx.EVT_BUTTON, self.V_calibrar_thread, self.button_V_calibrar)
        self.Bind(wx.EVT_BUTTON, self.go_back, self.button_I_back)
        self.Bind(wx.EVT_BUTTON, self.I_calibrar_thread, self.button_I_calibrar)
        self.Bind(wx.EVT_BUTTON, self.go_back, self.button_prog_back)
        self.Bind(wx.EVT_BUTTON, self.importar, self.button_importar)
        self.Bind(wx.EVT_BUTTON, self.imp_search, self.button_imp_search)
        self.Bind(wx.EVT_BUTTON, self.exportar, self.button_exportar)
        self.Bind(wx.EVT_BUTTON, self.exp_search, self.button_exp_search)
        self.Bind(wx.EVT_BUTTON, self.calibrar, self.button_prog_calibrar)
        self.Bind(wx.EVT_BUTTON, self.go_back, self.button_fase_back)
        self.Bind(wx.EVT_BUTTON, self.fase_calibrar_thread, self.button_fase_calibrar)
        self.Bind(wx.EVT_CLOSE, self.OnCloseWindow)
        
        self.calibracion = {"dir": "","slot": "", "fase": "", "medido": "", "tolerancia": "", "intentos": "", "VFS":"", "IFS":""}
        self.results = {"error": "", "hex": ""}
        self.direcciones = {"AMP_CC":[0x0,0x16],"VOLT_CC":[0x0,0x14], "PWR_CC":[0x0,0x18], "ENR_CC":[0x0,0x1A], "V_A": [0x1,0x32], "V_B": [0x2,0x1E], "V_C": [0x3,0x0A], "I_A": [0x1,0x30], "I_B": [0x2,0x1C], "I_C": [0x3,0x08], "I_N": [0x1,0x2E], "A_VRMS": [0x1,0xC8], "B_VRMS": [0x2,0xB4], "C_VRMS": [0x3,0xA0], "A_IRMS": [0x1,0xCC], "B_IRMS": [0x2,0xB8], "C_IRMS": [0x3,0xA4], "N_IRMS": [0x1,0x1C]}         
        self.reg = {"IFS": "", "VFS":""}
    # end wxGlade

    def __set_properties(self):
        # begin wxGlade: ConfigFrame.__set_properties
        self.SetTitle("Calibración MAXQ")
        self.button_reg_back.SetMinSize((85, 30))
        self.label_reg_volt.SetMinSize((135, 30))
        self.text_ctrl_reg_volt.SetMinSize((80, 30))
        self.label_reg_corriente.SetMinSize((135, 30))
        self.text_ctrl_reg_corriente.SetMinSize((80, 30))
        self.button_reg_calibrar.SetMinSize((170, 30))
        self.combo_box_V_fase.SetMinSize((180, 30))
        self.combo_box_V_fase.SetSelection(-1)
        self.button_V_back.SetMinSize((85, 30))
        self.label_volt_med.SetMinSize((180, 30))
        self.text_ctrl_volt_med.SetMinSize((80, 30))
        self.label_V.SetMinSize((20, 30))
        self.label_V_tol.SetMinSize((180, 30))
        self.text_ctrl_V_tol.SetMinSize((80, 30))
        self.label_V_percent.SetMinSize((20, 30))
        self.label_V_intent.SetMinSize((180, 30))
        self.text_ctrl_V_intent.SetMinSize((80, 30))
        self.button_V_calibrar.SetMinSize((85, 30))
        self.grid_voltaje.CreateGrid(5, 4)
        self.grid_voltaje.EnableEditing(0)
        self.grid_voltaje.SetColLabelValue(0, "Intentos")
        self.grid_voltaje.SetColLabelValue(1, "Error%")
        self.grid_voltaje.SetColLabelValue(2, "Gain")
        self.grid_voltaje.SetColLabelValue(3, "Hex")
        self.grid_voltaje.SetMinSize((365, 180))
        self.combo_box_I_fase.SetMinSize((180, 30))
        self.combo_box_I_fase.SetSelection(-1)
        self.button_I_back.SetMinSize((85, 30))
        self.label_I_med.SetMinSize((180, 30))
        self.text_ctrl_I_med.SetMinSize((80, 30))
        self.label_I.SetMinSize((20, 30))
        self.label_I_tol.SetMinSize((180, 30))
        self.text_ctrl_I_tol.SetMinSize((80, 30))
        self.label_I_percent.SetMinSize((20, 30))
        self.label_I_intent.SetMinSize((180, 30))
        self.text_ctrl_I_intent.SetMinSize((80, 30))
        self.button_I_calibrar.SetMinSize((85, 30))
        self.grid_corriente.CreateGrid(5, 4)
        self.grid_corriente.EnableEditing(0)
        self.grid_corriente.SetColLabelValue(0, "Intentos")
        self.grid_corriente.SetColLabelValue(1, "Error")
        self.grid_corriente.SetColLabelValue(2, "I_Gain")
        self.grid_corriente.SetColLabelValue(3, "Hex")
        self.grid_corriente.SetMinSize((365, 180))
        self.button_prog_back.SetMinSize((85, 30))
        self.text_ctrl_imp_path.SetMinSize((200, 30))
        self.button_imp_search.SetMinSize((30, 30))
        self.grid_respaldo.CreateGrid(13, 3)
        self.grid_respaldo.EnableEditing(0)
        self.grid_respaldo.SetColLabelValue(0, "Registro")
        self.grid_respaldo.SetColLabelValue(1, u"Dirección")
        self.grid_respaldo.SetColLabelValue(2, "Valor")
        self.grid_respaldo.SetCellValue(0,0,"AMP_CC")
        self.grid_respaldo.SetCellValue(1,0,"VOLT_CC")
        self.grid_respaldo.SetCellValue(2,0,"PWR_CC")
        self.grid_respaldo.SetCellValue(3,0,"ENR_CC")
        self.grid_respaldo.SetCellValue(4,0,"VA_GAIN")
        self.grid_respaldo.SetCellValue(5,0,"VB_GAIN")
        self.grid_respaldo.SetCellValue(6,0,"VC_GAIN")
        self.grid_respaldo.SetCellValue(7,0,"IA_GAIN")
        self.grid_respaldo.SetCellValue(8,0,"IB_GAIN")
        self.grid_respaldo.SetCellValue(9,0,"IC_GAIN")
        self.grid_respaldo.SetCellValue(10,0,"A_PA0")
        self.grid_respaldo.SetCellValue(11,0,"B_PA0")
        self.grid_respaldo.SetCellValue(12,0,"C_PA0")
        
        self.grid_respaldo.SetCellValue(0,1,"0x0016")
        self.grid_respaldo.SetCellValue(1,1,"0x0014")
        self.grid_respaldo.SetCellValue(2,1,"0x0018")
        self.grid_respaldo.SetCellValue(3,1,"0x001A")
        self.grid_respaldo.SetCellValue(4,1,"0x0132")
        self.grid_respaldo.SetCellValue(5,1,"0x021E")
        self.grid_respaldo.SetCellValue(6,1,"0x030A")
        self.grid_respaldo.SetCellValue(7,1,"0x0130")
        self.grid_respaldo.SetCellValue(8,1,"0x021C")
        self.grid_respaldo.SetCellValue(9,1,"0x0308")
        self.grid_respaldo.SetCellValue(10,1,"0x013E")
        self.grid_respaldo.SetCellValue(11,1,"0x022A")
        self.grid_respaldo.SetCellValue(12,1,"0x0316")
        
        self.combo_box_V_fase.SetSelection(0)
        self.combo_box_I_fase.SetSelection(0)
        self.grid_respaldo.SetMinSize((430, 220))
        self.text_ctrl_exp_path.SetMinSize((200, 30))
        self.button_exp_search.SetMinSize((30, 30))
        self.button_prog_calibrar.SetMinSize((85, 30))
        self.button_fase_back.SetMinSize((85, 30))
        self.label_fase_ang.SetMinSize((120, 30))
        self.text_ctrl_fase_ang.SetMinSize((120, 30))
        self.label_fase_grados.SetMinSize((30, 30))
        self.label_fase_carga.SetMinSize((120, 65))
        self.radio_box_fase_carga.SetMinSize((120, 65))
        self.radio_box_fase_carga.SetSelection(1)
        self.button_fase_calibrar.SetMinSize((85, 30))
        self.grid_fase.CreateGrid(5, 3)
        self.grid_fase.SetColLabelValue(0, "Ajuste")
        self.grid_fase.SetColLabelValue(1, "Error")
        self.grid_fase.SetColLabelValue(2, "Hex")
        self.grid_fase.SetMinSize((439, 170))
        # end wxGlade

    def __do_layout(self):
        # begin wxGlade: ConfigFrame.__do_layout
        sizer_12 = wx.BoxSizer(wx.VERTICAL)
        grid_sizer_9 = wx.FlexGridSizer(2, 1, 10, 0)
        grid_sizer_10 = wx.FlexGridSizer(4, 4, 10, 20)
        grid_sizer_6 = wx.FlexGridSizer(3, 1, 5, 0)
        grid_sizer_8 = wx.FlexGridSizer(1, 5, 0, 0)
        grid_sizer_7 = wx.FlexGridSizer(2, 5, 0, 0)
        grid_sizer_4_copy = wx.FlexGridSizer(2, 1, 10, 0)
        grid_sizer_5_copy = wx.FlexGridSizer(4, 4, 10, 20)
        grid_sizer_4 = wx.FlexGridSizer(2, 1, 10, 0)
        grid_sizer_5 = wx.FlexGridSizer(4, 4, 10, 20)
        grid_sizer_2 = wx.FlexGridSizer(9, 7, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((135, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((80, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((130, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add((135, 30), 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add((80, 30), 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add(self.button_reg_back, 0, wx.ALIGN_RIGHT, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add((20, 40), 0, 0, 0)
        grid_sizer_2.Add((135, 40), 0, 0, 0)
        grid_sizer_2.Add((20, 40), 0, 0, 0)
        grid_sizer_2.Add((80, 40), 0, 0, 0)
        grid_sizer_2.Add((20, 40), 0, 0, 0)
        grid_sizer_2.Add((130, 40), 0, 0, 0)
        grid_sizer_2.Add((20, 40), 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add(self.label_reg_volt, 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add(self.text_ctrl_reg_volt, 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add(self.label_V_V, 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((135, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((80, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((130, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add(self.label_reg_corriente, 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add(self.text_ctrl_reg_corriente, 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add(self.label_I_I, 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add((20, 60), 0, 0, 0)
        grid_sizer_2.Add((135, 60), 0, 0, 0)
        grid_sizer_2.Add((20, 60), 0, 0, 0)
        grid_sizer_2.Add((80, 60), 0, 0, 0)
        grid_sizer_2.Add((20, 60), 0, 0, 0)
        grid_sizer_2.Add((130, 60), 0, 0, 0)
        grid_sizer_2.Add((20, 60), 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add((135, 30), 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add((80, 30), 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add(self.button_reg_calibrar, 0, 0, 0)
        grid_sizer_2.Add((20, 30), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((135, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((80, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        grid_sizer_2.Add((130, 20), 0, 0, 0)
        grid_sizer_2.Add((20, 20), 0, 0, 0)
        self.notebook_registros.SetSizer(grid_sizer_2)
        grid_sizer_5.Add(self.combo_box_V_fase, 0, 0, 0)
        grid_sizer_5.Add((80, 30), 0, 0, 0)
        grid_sizer_5.Add((20, 30), 0, 0, 0)
        grid_sizer_5.Add(self.button_V_back, 0, 0, 0)
        grid_sizer_5.Add(self.label_volt_med, 0, 0, 0)
        grid_sizer_5.Add(self.text_ctrl_volt_med, 0, 0, 0)
        grid_sizer_5.Add(self.label_V, 0, 0, 0)
        grid_sizer_5.Add((85, 30), 0, 0, 0)
        grid_sizer_5.Add(self.label_V_tol, 0, 0, 0)
        grid_sizer_5.Add(self.text_ctrl_V_tol, 0, 0, 0)
        grid_sizer_5.Add(self.label_V_percent, 0, 0, 0)
        grid_sizer_5.Add((85, 30), 0, 0, 0)
        grid_sizer_5.Add(self.label_V_intent, 0, 0, 0)
        grid_sizer_5.Add(self.text_ctrl_V_intent, 0, 0, 0)
        grid_sizer_5.Add((20, 30), 0, 0, 0)
        grid_sizer_5.Add(self.button_V_calibrar, 0, 0, 0)
        grid_sizer_4.Add(grid_sizer_5, 1, wx.EXPAND, 0)
        grid_sizer_4.Add(self.grid_voltaje, 1, wx.EXPAND, 0)
        self.notebook_volt.SetSizer(grid_sizer_4)
        grid_sizer_5_copy.Add(self.combo_box_I_fase, 0, 0, 0)
        grid_sizer_5_copy.Add((80, 30), 0, 0, 0)
        grid_sizer_5_copy.Add((20, 30), 0, 0, 0)
        grid_sizer_5_copy.Add(self.button_I_back, 0, 0, 0)
        grid_sizer_5_copy.Add(self.label_I_med, 0, 0, 0)
        grid_sizer_5_copy.Add(self.text_ctrl_I_med, 0, 0, 0)
        grid_sizer_5_copy.Add(self.label_I, 0, 0, 0)
        grid_sizer_5_copy.Add((85, 30), 0, 0, 0)
        grid_sizer_5_copy.Add(self.label_I_tol, 0, 0, 0)
        grid_sizer_5_copy.Add(self.text_ctrl_I_tol, 0, 0, 0)
        grid_sizer_5_copy.Add(self.label_I_percent, 0, 0, 0)
        grid_sizer_5_copy.Add((85, 30), 0, 0, 0)
        grid_sizer_5_copy.Add(self.label_I_intent, 0, 0, 0)
        grid_sizer_5_copy.Add(self.text_ctrl_I_intent, 0, 0, 0)
        grid_sizer_5_copy.Add((20, 30), 0, 0, 0)
        grid_sizer_5_copy.Add(self.button_I_calibrar, 0, 0, 0)
        grid_sizer_4_copy.Add(grid_sizer_5_copy, 1, wx.EXPAND, 0)
        grid_sizer_4_copy.Add(self.grid_corriente, 1, wx.EXPAND, 0)
        self.notebook_corriente.SetSizer(grid_sizer_4_copy)
        grid_sizer_7.Add((30, 30), 0, 0, 0)
        grid_sizer_7.Add((85, 30), 0, 0, 0)
        grid_sizer_7.Add((200, 30), 0, 0, 0)
        grid_sizer_7.Add((30, 30), 0, 0, 0)
        grid_sizer_7.Add(self.button_prog_back, 0, 0, 0)
        grid_sizer_7.Add((30, 30), 0, 0, 0)
        grid_sizer_7.Add(self.button_exportar, 0, 0, 0)
        grid_sizer_7.Add(self.text_ctrl_exp_path, 0, 0, 0)
        grid_sizer_7.Add(self.button_exp_search, 0, 0, 0)
        grid_sizer_7.Add((85, 30), 0, 0, 0)
        grid_sizer_6.Add(grid_sizer_7, 1, wx.EXPAND, 0)
        grid_sizer_6.Add(self.grid_respaldo, 1, wx.EXPAND, 0)
        grid_sizer_8.Add((30, 30), 0, 0, 0)
        grid_sizer_8.Add(self.button_importar, 0, 0, 0)
        grid_sizer_8.Add(self.text_ctrl_imp_path, 0, 0, 0)
        grid_sizer_8.Add(self.button_imp_search, 0, 0, 0)
        grid_sizer_8.Add(self.button_prog_calibrar, 0, 0, 0)
        grid_sizer_6.Add(grid_sizer_8, 1, wx.EXPAND, 0)
        self.notebook_programar.SetSizer(grid_sizer_6)
        grid_sizer_10.Add((120, 30), 0, 0, 0)
        grid_sizer_10.Add((120, 30), 0, 0, 0)
        grid_sizer_10.Add((30, 30), 0, 0, 0)
        grid_sizer_10.Add(self.button_fase_back, 0, 0, 0)
        grid_sizer_10.Add(self.label_fase_ang, 0, 0, 0)
        grid_sizer_10.Add(self.text_ctrl_fase_ang, 0, 0, 0)
        grid_sizer_10.Add(self.label_fase_grados, 0, 0, 0)
        grid_sizer_10.Add((85, 30), 0, 0, 0)
        grid_sizer_10.Add(self.label_fase_carga, 0, 0, 0)
        grid_sizer_10.Add(self.radio_box_fase_carga, 0, 0, 0)
        grid_sizer_10.Add((30, 65), 0, 0, 0)
        grid_sizer_10.Add((85, 65), 0, 0, 0)
        grid_sizer_10.Add((120, 30), 0, 0, 0)
        grid_sizer_10.Add((120, 30), 0, 0, 0)
        grid_sizer_10.Add((30, 30), 0, 0, 0)
        grid_sizer_10.Add(self.button_fase_calibrar, 0, 0, 0)
        grid_sizer_9.Add(grid_sizer_10, 1, wx.EXPAND, 4)
        grid_sizer_9.Add(self.grid_fase, 1, wx.EXPAND, 0)
        self.notebook_fase.SetSizer(grid_sizer_9)
        self.notebook.AddPage(self.notebook_registros, "Registros")
        self.notebook.AddPage(self.notebook_volt, "Voltaje")
        self.notebook.AddPage(self.notebook_corriente, "Corriente")
        self.notebook.AddPage(self.notebook_fase, "Fase Offset")
        self.notebook.AddPage(self.notebook_programar, "Programar")
        self.notebook.AddPage(self.notebook_pane_ajustes, "Ajustes")
        sizer_12.Add(self.notebook, 1, wx.EXPAND, 0)
        self.SetSizer(sizer_12)
        sizer_12.Fit(self)
        self.Layout()
        self.Centre()
        # end wxGlade

    def OnCloseWindow(self, event):
        writeMAXQ('endcali')
        app.Destroy()
           
    def go_back(self, event): # wxGlade: ConfigFrame.<event_handler>
        print "Event handler `go_back' not implemented"
        self.Destroy()
        dialog.Show()
        event.Skip()

    def V_calibrar_thread(self,event):
        
        #open progress bar
        cc = threading.Thread(target = self.V_calibrar,args=[event])
        cc.start()
        
    def V_calibrar(self,event): # wxGlade: ConfigFrame.<event_handler>
        from struct import pack
        
        global MedBreak
        print '*'*37
        print '*'*10 + 'CALIBRACION VGAIN' + '*'*10
        print '*'*37
        MedBreak = True
        for fase_index in range(3):
            print '\tCalibrando fase: %s'%fase_index
            cali = {'fase':self.combo_box_V_fase.GetStringSelection(),
                    'ref':float(self.text_ctrl_volt_med.GetValue()),
                    'tolerancia':eval(self.text_ctrl_V_tol.GetValue()),
                    'intentos': self.text_ctrl_V_intent.GetValue()
                    }
            
            FASE_KEY = {0:'A',1:'B',2:'C'}[fase_index]
            
            cali['key_mu'] ='%s.VRMS'%FASE_KEY
            #cali['key_mu'] ='V.%s'%FASE_KEY
            cali['key_gain'] ='%s.VGAIN'%FASE_KEY
    
            #voltaje medido en lenguaje mu
            mu_medido = (float(self.text_ctrl_volt_med.GetValue()) * pow(2,24)) / self.reg["VFS"]
            #mu_medido = float(self.text_ctrl_volt_med.GetValue())
            
            
            #clear previous VGAIN value
            writeandcheckMAXQ(cali['key_gain'],[ord(i) for i in pack('>H',0x4000)],verbose=False)
            
            #calibracion iterada
            count = 0
            gain_past = 1
    
            while count < int(cali["intentos"]): 
                if readMAXQ(cali['key_mu']):
                    while len(SH.UART.bufferIn['params'][2:]) == 0:
                        pass
                    med_mu = convertToInt(SH.UART.bufferIn['params'][2:])
                    #med_mu = VirtualToStr(readMAXQ(cali['key_mu']))
                    error_mult = float(mu_medido/med_mu)
                     
                    error_porc = (error_mult) * 100
                    gain_now =  error_mult * gain_past
                    correccion_mult_mu = int(gain_now * pow(2,14))
                    correccion_mult_hex = hex(correccion_mult_mu)
                    gain_past = gain_now
                else:
                    print "error"
    
    
    
                print "\tError: %3.3g%%\tGAIN: 0x%X" % (abs(100-error_porc),correccion_mult_mu)
                #corregir VGAIN
                writeandcheckMAXQ(cali['key_gain'],[ord(i) for i in pack('>H',correccion_mult_mu)])
                self.grid_voltaje.SetCellValue(count, 0, str(count+1))
                self.grid_voltaje.SetCellValue(count, 1, '%3.4g'%abs(100-error_porc))
                self.grid_voltaje.SetCellValue(count, 2, '%3.4g'%gain_past)
                self.grid_voltaje.SetCellValue(count, 3, str(correccion_mult_hex))
                count = count + 1
                if abs(100-error_porc)<cali['tolerancia']:
                    self.grid_respaldo.SetCellValue(fase_index+4, 2, str(correccion_mult_hex))
                    break;

        MedBreak = False
        self.button_V_calibrar.Enable()

        #event.Skip()
    
    def I_calibrar_thread(self,event):
        #disable calibrate button
        self.button_I_calibrar.Disable()
        #open progress bar
        cc = threading.Thread(target = self.I_calibrar,args=[event])
        cc.start()
        
    def I_calibrar(self,event): # wxGlade: ConfigFrame.<event_handler>
        from struct import pack
        global MedBreak
        MedBreak = True
        
        print '*'*37
        print '*'*10 + 'CALIBRACION IGAIN' + '*'*10
        print '*'*37
        for fase_index in range(3):
            print '\tCalibrando fase: %s'%fase_index
            cali = {'fase':self.combo_box_I_fase.GetStringSelection(),
                    'ref':float(self.text_ctrl_I_med.GetValue()),
                    'tolerancia':eval(self.text_ctrl_I_tol.GetValue()),
                    'intentos': self.text_ctrl_I_intent.GetValue()
                    }
            
            FASE_KEY = {0:'A',1:'B',2:'C'}[fase_index]
            
            cali['key_mu'] ='%s.IRMS'%FASE_KEY
            cali['key_gain'] ='%s.IGAIN'%FASE_KEY
    
            #voltaje medido en lenguaje mu
            mu_medido = (float(self.text_ctrl_I_med.GetValue()) * pow(2,24)) / self.reg["IFS"]
            #mu_medido = float(self.text_ctrl_I_med.GetValue())
            
            #clear previous VGAIN value
            writeandcheckMAXQ(cali['key_gain'],[ord(i) for i in pack('>H',0x4000)],verbose=False)
                    
            #calibracion iterada
            count = 0
            gain_past = 1
    
            while count < int(cali["intentos"]): 
                if readMAXQ(cali['key_mu']):
                    while len(SH.UART.bufferIn['params'][2:]) == 0:
                        pass
                    med_mu = convertToInt(SH.UART.bufferIn['params'][2:])
                    error_mult = float(mu_medido/med_mu)
                    
                    error_porc = (error_mult) * 100
                    gain_now =  error_mult * gain_past
                    correccion_mult_mu = int(gain_now * pow(2,14))
                    correccion_mult_hex = hex(correccion_mult_mu)
                    gain_past = gain_now
                else:
                    print "error"
    
    
                print "\tError: %3.3g%%\tGAIN: 0x%X" % (abs(100-error_porc),correccion_mult_mu)
                #corregir VGAIN
                writeandcheckMAXQ(cali['key_gain'],[ord(i) for i in pack('>H',correccion_mult_mu)])
                self.grid_corriente.SetCellValue(count, 0, str(count+1))
                self.grid_corriente.SetCellValue(count, 1, '%3.4g'%error_porc)
                self.grid_corriente.SetCellValue(count, 2, '%3.4g'%gain_past)
                self.grid_corriente.SetCellValue(count, 3, str(correccion_mult_hex))
                count = count + 1
                if abs(100-error_porc)<cali['tolerancia']:                
                    self.grid_respaldo.SetCellValue(fase_index+7, 2, str(correccion_mult_hex))
                    break;

        
        self.button_I_calibrar.Enable()

        MedBreak = False
    
    #calibrar AMP CC y VOLT CC
    
    def reg_calibrar(self,event): # wxGlade: ConfigFrame.<event_handler>
        global MedBreak
        MedBreak = True
        #self.calibracion["dir"] = self.nodos["dir"]
        #self.calibracion["slot"] = self.nodos["slot"]
        I_RShunt = 0.0529
        MAX_ADC_VOLT = 1.024
        AMP_LSB = 0.1
        VOLT_LSB = 0.1
        PWR_LSB = 0.1
        #ENR_LSB = 0.1
        ENR_LSB = 0.1
        TFR = 8.89 * pow(10,-8)
        
        IFS = float(self.text_ctrl_reg_corriente.GetValue()) * I_RShunt
        print "IFS %s"%IFS
        #IFS = 174,57
        AMP_CC = IFS / pow(2,24) / (AMP_LSB / pow(2,(32-16)))
        correctionPARAMS['AMP_CC'] = float(AMP_CC/int(AMP_CC))
        print "AMP_CC %s"%AMP_CC
        self.grid_respaldo.SetCellValue(0,2,"0x%x" % AMP_CC)
        
        VFS = float(self.text_ctrl_reg_volt.GetValue()) * MAX_ADC_VOLT
        print "VFS %s"%VFS 
        #VFS = 553,984
        VOLT_CC = VFS / pow(2,24) / (VOLT_LSB / pow(2,(32-16)))
        correctionPARAMS['VOLT_CC'] = float(VOLT_CC/int(VOLT_CC))
        print "VOLT_CC %s"%VOLT_CC
        
        self.grid_respaldo.SetCellValue(1,2,"0x%x" % VOLT_CC)
        
        PWR_CC = VFS * IFS / (pow(2,32) * PWR_LSB / pow(2,32-16))
        correctionPARAMS['PWR_CC'] = float(PWR_CC/int(PWR_CC))
        self.grid_respaldo.SetCellValue(2,2,"0x%x" % PWR_CC)
        print "PWR_CC %s"%PWR_CC
        
        ENR_CC = float(IFS * VFS * TFR / (1.0* pow(2,16) * ENR_LSB / pow(2,32-8)))
        correctionPARAMS['ENR_CC'] = float(ENR_CC/int(ENR_CC))
        print "ENR_CC %s"%ENR_CC
        self.grid_respaldo.SetCellValue(3,2,"0x%x" % ENR_CC)
        
        self.reg["IFS"] = IFS
        self.reg["VFS"] = VFS
        
        #send parameters to MAXQ
        
        print 'Check VOLT_C: %s'%{True:'OK',False:'Fallo'}[writeandcheckMAXQ('VOLT_CC',[int(VOLT_CC)>>8,int(VOLT_CC)&0x00ff],verbose=False)]
        
        print 'Check AMP_C: %s'%{True:'OK',False:'Fallo'}[writeandcheckMAXQ('AMP_CC',[int(AMP_CC)>>8,int(AMP_CC)&0x00ff],verbose=False)]
        
        print 'Check PWR_C: %s'%{True:'OK',False:'Fallo'}[writeandcheckMAXQ('PWR_CC',[int(PWR_CC)>>8,int(PWR_CC)&0x00ff],verbose=False)]
        
        print 'Check ENR_C: %s'%{True:'OK',False:'Fallo'}[writeandcheckMAXQ('ENR_CC',[int(ENR_CC)>>8,int(ENR_CC)&0x00ff],verbose=False)]
        
        self.button_reg_calibrar.Enable()
            
        MedBreak = False
        #event.Skip()

    def reg_calibrar_thread(self,event):
        #disable calibrate button
        self.button_reg_calibrar.Disable()
        
        #open progress bar
        cc = threading.Thread(target = self.reg_calibrar,args=[event])
        cc.start()
        #self.button_reg_calibrar.Disable()
        
    def fase_calibrar_thread(self,event):
        #disable calibrate button
        self.button_fase_calibrar.Disable()
        
        #open progress bar
        cc = threading.Thread(target = self.fase_calibrar,args=[event])
        cc.start()
        #self.button_reg_calibrar.Disable()
        
    def fase_calibrar(self, event): # wxGlade: ConfigFrame.<event_handler>
        global MedBreak
        from struct import pack
        MedBreak = True
        print '*'*37
        print '*'*10 + 'CALIBRACION FASE' + '*'*10
        print '*'*37
        for fase_index in range(3):
            print '\tCalibrando fase: %s'%fase_index
            cali = {}
            KEY_PHASE = {0:'A',1:'A',2:'A'}[fase_index]
            
            cali['PA0_key'] = '%s.PA0'%KEY_PHASE
            cali['PA1_key'] = '%s.PA1'%KEY_PHASE
            cali['PA2_key'] = '%s.PA2'%KEY_PHASE
            cali['ACT_mu'] = '%s.ACT'%KEY_PHASE
            cali['REA_mu'] = '%s.REA'%KEY_PHASE
            #Limpiar registos
            writeandcheckMAXQ('I1THR',[ord(i) for i in pack('>H',0x0000)],verbose=False)
            writeandcheckMAXQ('I2THR',[ord(i) for i in pack('>H',0x0000)],verbose=False)
            writeandcheckMAXQ(cali['PA0_key'],[ord(i) for i in pack('>H',0x0000)],verbose=False)
            writeandcheckMAXQ(cali['PA1_key'],[ord(i) for i in pack('>H',0x0000)],verbose=False)
            writeandcheckMAXQ(cali['PA2_key'],[ord(i) for i in pack('>H',0x0000)],verbose=False)
            
            #mediciones de potencia 
            #calibracion iterada
             
            if readMAXQ(cali['ACT_mu']):
                while len(SH.UART.bufferIn['params'][2:]) == 0:
                    pass
                active_med_mu = convertToInt(SH.UART.bufferIn['params'][2:])
                if readMAXQ(cali['REA_mu']):
                    while len(SH.UART.bufferIn['params'][2:]) == 0:
                        pass
                    reactive_med_mu = convertToInt(SH.UART.bufferIn['params'][2:])
                    
                    if int(reactive_med_mu) & 0x80000000:
                        type = 1
                        reactive_med_mu = ((int(reactive_med_mu)^0xffffffff) + 1)*(-1)
                    else:
                        type = 0
    
                    leido = math.atan2(reactive_med_mu, active_med_mu)
                    fp = math.cos(leido)
                    leido = math.degrees(leido)
                    if self.radio_box_fase_carga.GetSelection():
                        adj = eval(self.text_ctrl_fase_ang.GetValue()) - leido
                    else:
                        adj = eval(self.text_ctrl_fase_ang.GetValue()) + leido
                    
                    if adj > 0:
                        adj_hex = math.radians(adj)*65536
                    else:
                        adj_hex = (int(math.radians(adj*(-1))*(-65536))&0xffffffff)+1
                    
                    
                    writeandcheckMAXQ(cali['PA0_key'],[ord(i) for i in pack('>H',adj_hex)])                    
                    ant = adj_hex
                    print "activa: %x, reactiva: %x, fp: %f, ang: %f, adj: %f, hex: %x" % (active_med_mu, reactive_med_mu, fp, leido, adj, adj_hex)
                    i=0    
                    error = 5
                    #comienzo de Algoritmo de Newton
                    while True:
                        
                        if readMAXQ(cali['ACT_mu']):
                            while len(SH.UART.bufferIn['params'][2:]) == 0:
                                pass
                            active_med_mu = convertToInt(SH.UART.bufferIn['params'][2:])
                            if readMAXQ(cali['REA_mu']):
                                while len(SH.UART.bufferIn['params'][2:]) == 0:
                                    pass
                                reactive_med_mu = convertToInt(SH.UART.bufferIn['params'][2:])
                                
                                if int(reactive_med_mu) & 0x80000000:
                                    type = 1
                                    reactive_med_mu = ((int(reactive_med_mu)^0xffffffff) + 1)*(-1)
                                else:
                                    type = 0
            
                                leido = math.atan2(reactive_med_mu, active_med_mu)
                                fp = math.cos(leido)
                                leido = math.degrees(leido)
                        
                                if self.radio_box_fase_carga.GetSelection():
                                    adj = eval(self.text_ctrl_fase_ang.GetValue()) - leido
                                else:
                                    adj = eval(self.text_ctrl_fase_ang.GetValue()) + leido
                                if adj > 0:
                                    adj_hex = math.radians(adj)*65536
                                else:
                                    adj_hex = (int(math.radians(adj*(-1))*(-65536))&0xffffffff)+1
                                
                                if ant < 0:
                                    ant = int(ant)&0xffffffff+1
                                error = eval(self.text_ctrl_fase_ang.GetValue()) - leido
                                #derror = int(error-errora)/int(adj_hex-ant)
                                print "error %d" % error
                                if (i>10) or ((error < 0.2) and (error > -0.2)):
                                    print "\n\tTermino de busqueda, error:%2.2g°\n"%error
                                    self.grid_respaldo.SetCellValue(fase_index+10, 2, '0x%x'%adj_hex) 
                                    break
                                if error < 0 :
                                    adj_hex = ant - 64
                                else:
                                    adj_hex = ant + 64
                                if adj_hex < 0:
                                    adj_hex = int(adj_hex)&0xffffffff+1
                                    
                                writeandcheckMAXQ(cali['PA0_key'],[ord(j) for j in pack('>H',adj_hex)])           
                    
                                ant = adj_hex
                                print "activa: %x, reactiva: %x, fp: %f, ang: %f, adj: %f, hex: %x" % (active_med_mu, reactive_med_mu, fp, leido, adj, adj_hex)
                                i = i + 1

            
                        
            
        MedBreak = False
        event.Skip()
        
    
    def imp_search(self, event): # wxGlade: ConfigFrame.<event_handler>
        filename = ""
        dlg = wx.FileDialog(self, message="Choose a file")
        if dlg.ShowModal() == wx.ID_OK:
            filename = dlg.GetPath()
        dlg.Destroy()
        self.text_ctrl_imp_path.Value = filename
        event.Skip()
        
    def exp_search(self, event): # wxGlade: ConfigFrame.<event_handler>
        filename = ""
        dlg = wx.FileDialog(self, message="Choose a file")
        if dlg.ShowModal() == wx.ID_OK:
            filename = dlg.GetPath()
        dlg.Destroy()
        self.text_ctrl_exp_path.Value = filename
        event.Skip()

    def exportar(self, event): # wxGlade: ConfigFrame.<event_handler>
        if self.text_ctrl_exp_path.GetValue() == "":
            print "Carpeta no encontrada"
        else:
            respaldo = open(self.text_ctrl_exp_path.GetValue(),'w')
            j = 0
            k = 0
            for i in range(39):
                
                respaldo.write(self.grid_respaldo.GetCellValue(k,j))
                if j == 2:
                    respaldo.write('\n')
                    j = 0
                    k = k + 1
                else:
                    respaldo.write('\t')
                    j = j + 1
            respaldo.close()
        event.Skip()
        
    def importar(self, event): # wxGlade: ConfigFrame.<event_handler>
        if self.text_ctrl_imp_path.GetValue() == "":
            print "Archivo no encontrado"
        else:
            self.grid_respaldo.ClearGrid()    
            respaldo = open(self.text_ctrl_imp_path.GetValue(), 'r')
            respaldo.seek(0,2)
            len = respaldo.tell()
            respaldo.seek(0)
            buff = ""
            buff2 = ""
            j = 0
            k = 0
            i = 0
            while i < len:
                buff = respaldo.read(1)
                if buff == '\t':
                    self.grid_respaldo.SetCellValue(k,j,buff2)
                    buff2 = ""
                    j = j + 1
                elif buff == '\n':
                    self.grid_respaldo.SetCellValue(k,j,buff2)
                    buff2 = ""
                    j = 0
                    k = k + 1
                else:
                    buff2 = buff2 + buff
                i = i + 1
            respaldo.close()
        event.Skip()
        
    def calibrar(self, event): # wxGlade: ConfigFrame.<event_handler>
        from struct import pack
        global MedBreak
        MedBreak = True
        
        method = 100
        id = 99
        
        
        MAX_BYTES = 68
        valid_params = 0 
        for i in range(13):
            if self.grid_respaldo.GetCellValue(i,2) != "":
                valid_params = valid_params + 1
        
        if divmod(valid_params * 5, MAX_BYTES-1)[1]>0:
            num_packet = 1
            
        num_packets = num_packet + divmod(valid_params * 5, MAX_BYTES-1)[0] 
        
        last_index = 0
        for j in range(num_packets):
            params = [last_index] # first elem indicate cali index params, this allow to append more parameter to the eeprom buffer. Send the entire calibration stream in several packets
            for i in range(last_index,13):
                if self.grid_respaldo.GetCellValue(i,2) != "":
                    if len(params) + 5 < MAX_BYTES:
                        params = params + [ord(m) for m in pack('>H',eval(self.grid_respaldo.GetCellValue(i,1)))] + [ord(n) for n in pack('>H',eval(self.grid_respaldo.GetCellValue(i,2)))] + [2]
                    else:
                        last_index = i+1
                        break;
            pkg = {'method':method,'params':params,'id':id}
            writePkg(pkg)
                    
        # release maxq node
        METHOD = 230
        params = [0]
        packet_id = 88
        
        pkg = {'method':METHOD,'params':params,'id':packet_id}
        writePkg(pkg)
        
        MedBreak = False
        event.Skip()


# end of class ConfigFrame


lock = threading.Lock()
MedBreak = False

if __name__ == "__main__":
    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    SH  =   SerialHandling()
    dialog = SerialSelect(None, -1, "")
    dialog.Show()
    app.SetTopWindow(dialog)
    app.MainLoop()