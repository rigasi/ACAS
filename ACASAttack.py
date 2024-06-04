from XPPython3 import xp #import XPPython for communicating with xplane
import math #for the calculations
import configparser #for reading the ini file
from utilities import * 
from XPPython3.utils.widgetMsgHelper import WidgetMsgHelper #for the widget
import csv #for writing to CSV

class PythonInterface:

    def __init__(self):
        self.Name = "ACAS experiment v1.0"
        self.Sig = "iason.rigas.thesis"
        self.Desc = "Plugin for ACAS attack testing"
        self.myWidgetWindow = None
        self.plugin_owns_tcas = False #variable to store whether we own TCAS in the sim
        self.setup_csv_logging()  # Setup CSV logging of data
        self.initialize_variables()

    def initialize_variables(self):
            self.fttomtr = 0.3048
            self.nmtomtr = 1852.0
            self.TARGET = 1 #number of targets we will be using in the sim
            self.ids = [0xA41B14]  # Assign a 24bit ICAO address to the ghost aircraft.
            self.tailnum = [b"D-EHNR"]  # Assign a flight id to the ghost.
            #Initialise all variables
            self.attacker_slant = 0
            self.ghost_lat = 0
            self.ghost_lon = 0
            self.ghost_elevation = 950
            self.ghost_slant = 0
            self.effective_angle = 0

            self.target_lat = None
            self.target_lon = None
            self.target_elevation = None
            self.target_speed = 0
            self.target_trk = 0
            self.ghost_rbearing = 0
            self.ghost_raltitude = 0
            self.ghost_rdistance = 0
            self.closing_speed = 0

            self.ghost_heading = 0
            self.ghost_bearing=0

            self.attack_valid = True
            self.RA_triggered=False
            self.elevation_angle=0
            self.effective_angle=0
            self.target_theta=0
            self.ghost_rbearing=0
            self.ghost_raltitude=0 
            self.ghost_rdistance=0
            self.closing_speed=0
            self.current_time=0

            # Initialize xplane datarefs
            self.initialize_datarefs()

    def initialize_datarefs(self):
            self.ref_target_trk = xp.findDataRef("sim/flightmodel/position/true_psi")
            self.ref_target_elevation = xp.findDataRef("sim/flightmodel/position/elevation")
            self.ref_vx = xp.findDataRef("sim/flightmodel/position/local_vx")
            self.ref_vy = xp.findDataRef("sim/flightmodel/position/local_vy")
            self.ref_target_latitude = xp.findDataRef("sim/flightmodel/position/latitude")
            self.ref_target_longitude = xp.findDataRef("sim/flightmodel/position/longitude")
            self.ref_target_gs = xp.findDataRef("sim/flightmodel/position/groundspeed")
            self.ref_target_theta = xp.findDataRef("sim/flightmodel/position/theta")
            self.ref_ghost_rbrg = xp.findDataRef("sim/cockpit2/tcas/indicators/relative_bearing_degs")
            self.ref_ghost_rdis = xp.findDataRef("sim/cockpit2/tcas/indicators/relative_distance_mtrs")
            self.ref_ghost_ralt = xp.findDataRef("sim/cockpit2/tcas/indicators/relative_altitude_mtrs")
            self.ref_modeS_id = xp.findDataRef("sim/cockpit2/tcas/targets/modeS_id")  
            self.ref_flt_id = xp.findDataRef("sim/cockpit2/tcas/targets/flight_id")  
            self.ref_override = xp.findDataRef("sim/operation/override/override_TCAS")
            self.glat = xp.findDataRef("sim/cockpit2/tcas/targets/position/x")
            self.glon = xp.findDataRef("sim/cockpit2/tcas/targets/position/y")
            self.gele = xp.findDataRef("sim/cockpit2/tcas/targets/position/z")

    def initialise_attack(self):
        # Setting the initial target data by reading the xplane datarefs for the aircraft we are flying
        self.target_lat = xp.getDatad(self.ref_target_latitude)
        self.target_lon = xp.getDatad(self.ref_target_longitude)
        self.target_elevation = xp.getDatad(self.ref_target_elevation) #the target altitude is displayed as elevation in meters

        # Calculate the bearing from the attacker to the target, this is used to correctly plot a convincing ghost
        self.ghost_heading = calculate_required_heading(self.attacker_lat, self.attacker_lon, self.target_lat, self.target_lon)
        ghost_initial_bearing = (self.ghost_heading + 180) % 360  # Ghost initial bearing is the opposite direction
        # Calculate initial position for the ghost using the start distance and ghost bearing
        # The start distance is an arbitrary value which ensures that the ghost will be displayed on the target TCAS for a few seconds before the attack
        self.ghost_lat, self.ghost_lon = calculate_initial_ghost_position(self.attacker_lat, self.attacker_lon, self.start_distance, ghost_initial_bearing)
    
    """Updates in each simulator frame the new position of the ghost and calculates some parameters"""
    def update_dists_callback(self, elapsed1, elapsed2, counter, refcon):
        speed_mps = 0.514444 * self.ghost_speed #calculations are based on meters per second but operationally knots are used
        distance_per_frame = speed_mps * elapsed1 #calculate the distance moved in this frame
        heading_radians = math.radians(self.ghost_heading) #convert to radians for the calculations
        delta_lat = (distance_per_frame * math.cos(heading_radians)) / 111139 #calculate difference in latitude
        delta_lon = (distance_per_frame * math.sin(heading_radians)) / (111139 * math.cos(math.radians(self.ghost_lat))) #calculate difference in longitude
        self.ghost_lat= self.ghost_lat + delta_lat
        self.ghost_lon= self.ghost_lon + delta_lon
        self.check_proximity() #check if the attack geometry is still valid
        #read target position and data
        self.target_lat=xp.getDatad(self.ref_target_latitude)
        self.target_lon=xp.getDatad(self.ref_target_longitude)
        self.target_elevation=xp.getDatad(self.ref_target_elevation)
        self.target_speed = xp.getDataf(self.ref_target_gs)* 1.94384
        self.target_trk = xp.getDataf(self.ref_target_trk)      
        #update the elevation angle , effective angle (considers aircraft pitch theta), and new ghost heading 
        self.elevation_angle = calculate_elevation_angle(self.attacker_lat, self.attacker_lon, self.attacker_elevation, self.target_lat, self.target_lon, self.target_elevation)
        self.target_theta = xp.getDataf(self.ref_target_theta)
        self.effective_angle=self.target_theta+self.elevation_angle
        self.ghost_heading = calculate_required_heading(self.ghost_lat, self.ghost_lon, self.target_lat, self.target_lon)

        return -1  # callback to run again in the next frame

    """check if the attack geometry is still via slant range comparisons, and calculate some additional variables"""
    def check_proximity(self):
        #calculate slant range between attacker and target
        self.attacker_slant = haversine(self.attacker_lat, self.attacker_lon, self.target_lat, self.target_lon,self.attacker_elevation,self.target_elevation)
        #calculate slant range between ghost and target
        self.ghost_slant = haversine(self.ghost_lat, self.ghost_lon, self.target_lat, self.target_lon, self.ghost_elevation, self.target_elevation)
        #if the ghost slant range becomes equal to the attacker slant range the ghost is too close to the target and the 
        #attack geometry is not valid anymore.
        if self.ghost_slant <= self.attacker_slant:
            self.attack_valid = False
        #Read the relative bearing, altitude and distance between the ghost and the target from the simulator
        values_rbearing=[]
        values_raltitude=[]
        values_rdistance=[]
        xp.getDatavf(self.ref_ghost_rbrg,values_rbearing)
        xp.getDatavf(self.ref_ghost_ralt,values_raltitude)
        xp.getDatavf(self.ref_ghost_rdis,values_rdistance)
        self.ghost_rbearing=values_rbearing[1]
        self.ghost_raltitude=values_raltitude[1]   
        self.ghost_rdistance=values_rdistance[1]
        #calculate the closing speed between ghost and target in knots
        self.closing_speed=calculate_relative_speed_in_knots(self.ghost_speed,self.ghost_heading,self.target_speed,self.target_trk)

    """update the position of the ghost aircraft on the TCAS on each frame"""
    def update_tcas(self, elapsed1, elapsed2, counter, refcon):

        for i in range(self.TARGET):   
            self.ghost_elevation=xp.getDatad(self.ref_target_elevation)
            llat=[self.ghost_lat]
            llon=[self.ghost_lon]
            lalt=[self.ghost_elevation]  
            #we need to convert the lat,lon,and altitude from the normal world coordinate system to the local simulator system (OpenGL system)
            x, y, z = xp.worldToLocal(llat[i], llon[i], lalt[i])   
            gx=[x]
            gy=[y]
            gz=[z]
        #if our plugin owns the tcas and no other process is involved then we set the coordinates
        if self.plugin_owns_tcas:
            xp.setDatavf(self.glat,gx,1,self.TARGET)    
            xp.setDatavf(self.glon,gy,1,self.TARGET)
            xp.setDatavf(self.gele,gz,1,self.TARGET)
        #if the attack geometry is not valid the target must be removed from the TCAS display immediately
        if self.attack_valid==False:
            return 0 #drop the target
        return -1  # run again in the next frame
    
    """the following functions are required for all xplane plugins"""
    def XPluginStart(self):
        return self.Name, self.Sig, self.Desc
    
    def XPluginEnable(self):
        self.myWidgetWindow = self.create_widget_window()
        return 1

    def XPluginDisable(self):
        if self.myWidgetWindow:
            xp.destroyWidget(self.myWidgetWindow['widgetID'], 1)
            self.myWidgetWindow = None

        if self.plugin_owns_tcas:
            self.not_our_planes()

    def retry_acquiring_planes(self, ignored):
        if not xp.acquirePlanes(None, self.retry_acquiring_planes, None):
            xp.debugString("The plugin could not acquire the planes\n")
        else:
            self.my_tcas()
    
    def my_tcas(self):
        #use the override to control the Xplane TCAS display
        xp.setDatai(self.ref_override, 1)
        #ensure that the number of aircraft we are writint to TCAS is less than the maximum
        max_targets = xp.getDatavi(self.ref_modeS_id, None, 0, 0)
        assert self.TARGET < max_targets
        xp.setActiveAircraftCount(self.TARGET)
        self.plugin_owns_tcas = True
        #set our ghost details
        xp.setDatavi(self.ref_modeS_id, self.ids, 1, self.TARGET)
        for i in range(1, self.TARGET + 1):
            xp.setDatab(self.ref_flt_id, self.tailnum[i - 1], i * 8, len(self.tailnum[i - 1]))
        #start the attack
        self.initialise_attack()
        xp.registerFlightLoopCallback(self.update_tcas, 0.1, None)
        xp.registerFlightLoopCallback(self.update_dists_callback, 0.1, None)
        xp.registerFlightLoopCallback(self.log_data_to_csv, 0.1, None)

    def not_our_planes(self):
        #if we lose countrol of the planes unregister the callbacks remove the override and release the planes
        xp.unregisterFlightLoopCallback(self.update_tcas, None)
        xp.unregisterFlightLoopCallback(self.update_dists_callback, None)
        xp.unregisterFlightLoopCallback(self.log_data_to_csv, None)
        xp.setDatai(self.ref_override, 0)
        xp.releasePlanes()
        self.plugin_owns_tcas = False

    """Prepare the csv logging , initialise file and headers"""
    def setup_csv_logging(self):
     
        self.csv_file = open('TCAS_Data_Log.csv', 'w', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_file)
        headers = ["Time", "Attacker Latitude", "Attacker Longitude", "Attacker Elevation","Attacker Slant",
                   "Ghost Latitude", "Ghost Longitude", "Ghost Elevation","Ghost Speed","Ghost Heading","Ghost Slant",
                   "Ghost Bearing","Ghost Alt","Ghost Dist","Target Lat","Target Lon","Target Elevation","Target Speed",
                   "Target Heading","Effective Angle","Closing Speed","Attack Valid","RA Triggered"]
        self.csv_writer.writerow(headers)
        self.start_time = xp.getElapsedTime()

    """This is used to write our sim data to the csv file for later analysis"""
    def log_data_to_csv(self, elapsed1, elapsed2, counter, refcon):
        if self.csv_file.closed:
            return 0  # Stop this callback if the file is closed
    
        self.current_time = xp.getElapsedTime() - self.start_time
        #write the following fields
        data = [
            self.current_time,
            self.attacker_lat,
            self.attacker_lon,
            self.attacker_elevation,
            self.attacker_slant,
            self.ghost_lat,
            self.ghost_lon,
            self.ghost_elevation,
            self.ghost_speed,
            self.ghost_heading,
            self.ghost_slant,
            self.ghost_rbearing,
            self.ghost_raltitude,
            self.ghost_rdistance,
            self.target_lat,
            self.target_lon,
            self.target_elevation,
            self.target_speed,
            self.target_trk,
            self.effective_angle,
            self.closing_speed,
            self.attack_valid,
            self.RA_triggered
        ]
        #write the row
        self.csv_writer.writerow(data)
        self.csv_file.flush()  # Ensure data is written to the file system
        #if the attack geometry is not valid anymore stop writing data and close th file
        if self.attack_valid is False:
            try:
                self.csv_file.close()
            except Exception as e:
                xp.debugString(f"Error closing file: {str(e)}\n")
            return 0
           
        return 0.1  # Call this function again in 0.1 seconds



    def XPluginReceiveMessage(self, msg_from, msg, param):
        if msg == MSG_RELEASE_PLANES:
            # if another plugin needs to take control of our tcas planes release them
            self.not_our_planes()
            who = xp.getPluginInfo(msg_from)

    """create the widget we will use for controlling our simulation"""
    def create_widget_window(self):
        widgetWindow = {'widgetID': None,  # the ID of the main window containing all other widgets
                            'widgets': {}      # dict() of all child widgets we care about
            }
        
        top = 700
        left = 100
        right = 400
        bottom = 1

        widgetWindow['widgetID'] = xp.createWidget(left, top, right, bottom, 1, "TCAS Attack Control",
                                                   1, 0, xp.WidgetClass_MainWindow)
        xp.addWidgetCallback(widgetWindow['widgetID'], self.widgetCallback)

        # Define initial positions for labels and text fields
        vertical_spacing = 25
        current_top = top - 40
        labels = [
            "Attacker lat", "Attacker lon", "Attacker elevation", "Attacker slant",
            "Ghost lat", "Ghost lon", "Ghost elevation", "Ghost speed", "Ghost heading", "Ghost slant",
            "Ghost rel.bearing", "Ghost rel.alt", "Ghost rel.dist", "Target lat", "Target lon",
            "Target elevation", "Target speed", "Target heading", "Elevation angle", "Closing speed (kt):",
            "Attack Valid:", "RA:"
        ]
        text_fields = []

        for label in labels:
            label_id = xp.createWidget(left + 10, current_top, left + 120, current_top - 20, 1, label, 0, widgetWindow['widgetID'], xp.WidgetClass_Caption)
            text_field_id = xp.createWidget(left + 130, current_top, right - 10, current_top - 20, 1, "", 0, widgetWindow['widgetID'], xp.WidgetClass_TextField)
            text_fields.append(text_field_id)
            current_top -= vertical_spacing

        self.widget_dict = {
            "att_lat": text_fields[0],
            "att_lon": text_fields[1],
            "att_elevation": text_fields[2],
            "att_slant": text_fields[3],
            "ghost_lat": text_fields[4],
            "ghost_lon": text_fields[5],
            "ghost_elevation": text_fields[6],
            "ghost_speed": text_fields[7],
            "ghost_heading": text_fields[8],
            "ghost_slant": text_fields[9],
            "ghost_rbearing": text_fields[10],
            "ghost_raltitude": text_fields[11],
            "ghost_rdistance": text_fields[12],
            "target_lat": text_fields[13],
            "target_lon": text_fields[14],
            "target_elevation": text_fields[15],
            "target_speed": text_fields[16],
            "target_heading": text_fields[17],
            "effective_angle": text_fields[18],
            "closing_speed": text_fields[19],
            "attack_valid": text_fields[20],
            "RA": text_fields[21],
         }  

        # Create buttons
        attack_button_top = bottom + 50
        attack_button_bottom = bottom +10
        attack_button_left = left + 20
        attack_button_right = left + 120
        ra_button_top = bottom + 50
        ra_button_bottom = bottom + 10
        ra_button_left = left + 130
        ra_button_right = left + 230

        #create two buttons, one for starting the attack and one for manually marking the RA
        self.launch_button = xp.createWidget(attack_button_left, attack_button_top, attack_button_right, attack_button_bottom, 1, "Launch Attack", 0, widgetWindow['widgetID'], xp.WidgetClass_Button)
        xp.setWidgetProperty(self.launch_button, xp.Property_ButtonType, xp.PushButton) 
        self.RA_button = xp.createWidget(ra_button_left, ra_button_top, ra_button_right, ra_button_bottom, 1, "RA", 0, widgetWindow['widgetID'], xp.WidgetClass_Button)
        xp.setWidgetProperty(self.RA_button, xp.Property_ButtonType, xp.PushButton) 
        #load data from the config file
        self.loadConfig()
        #start updating the widget
        xp.registerFlightLoopCallback(self.update_widget_fields, 0.1, None)
    
    """load our ini data"""
    def loadConfig(self):
        self.config = configparser.ConfigParser()
        self.config.read('config.ini')
        self.load_initial_settings()

    """read the file and set the variables"""
    def load_initial_settings(self):
        settings = self.config['Settings']
        self.attacker_lat = float(settings['attacker_lat'])
        self.attacker_lon = float(settings['attacker_lon'])
        self.attacker_elevation = float(settings['attacker_elevation'])
        self.start_distance = float(settings['start_distance'])
        self.ghost_speed = float(settings['ghost_speed'])

    """parse the float"""
    def parse_float(self, value):
        try:
            return float(value)
        except ValueError:
            return None
        
    """callback for handling widget actions and detecting if one of the interface buttons is pressed"""
    def widgetCallback(self, inMessage, inWidget, inParam1, inParam2):  
        if (inMessage == xp.Msg_PushButtonPressed):
             if (inParam1 == self.launch_button):  
                self.startAttack()
                return 1
        if (inMessage == xp.Msg_PushButtonPressed):
             if (inParam1 == self.RA_button):  
                self.RA_triggered=True
                return 1
        return 0
    
    """if a the start button is pressed start the attack"""
    def startAttack(self):
        if not xp.acquirePlanes(None, self.retry_acquiring_planes, None):
            (total, active, controller) = xp.countAircraft()
            who = xp.getPluginInfo(controller)
            xp.debugString("The plugin could acquire the TCAS because it is used by another plugin")
        else:
            self.my_tcas()

    def update_widget_fields(self, elapsed1, elapsed2, counter, refcon):
        # Update widget fields based on the latest data
        xp.setWidgetDescriptor(self.widget_dict['att_lat'], str(self.attacker_lat))
        xp.setWidgetDescriptor(self.widget_dict['att_lon'], str(self.attacker_lon))
        xp.setWidgetDescriptor(self.widget_dict['att_elevation'], str(self.attacker_elevation))
        xp.setWidgetDescriptor(self.widget_dict['att_slant'], str(self.attacker_slant))
        xp.setWidgetDescriptor(self.widget_dict['ghost_lat'], str(self.ghost_lat))
        xp.setWidgetDescriptor(self.widget_dict['ghost_lon'], str(self.ghost_lon))
        xp.setWidgetDescriptor(self.widget_dict['ghost_elevation'], str(self.ghost_elevation))
        xp.setWidgetDescriptor(self.widget_dict['ghost_speed'], str(self.ghost_speed))
        xp.setWidgetDescriptor(self.widget_dict['ghost_heading'], str(self.ghost_heading))
        xp.setWidgetDescriptor(self.widget_dict['ghost_slant'], str(self.ghost_slant))
        xp.setWidgetDescriptor(self.widget_dict['ghost_rbearing'], str(self.ghost_rbearing))
        xp.setWidgetDescriptor(self.widget_dict['ghost_raltitude'], str(self.ghost_raltitude))
        xp.setWidgetDescriptor(self.widget_dict['ghost_rdistance'], str(self.ghost_rdistance))
        xp.setWidgetDescriptor(self.widget_dict['target_lat'], str(self.target_lat))
        xp.setWidgetDescriptor(self.widget_dict['target_lon'], str(self.target_lon))
        xp.setWidgetDescriptor(self.widget_dict['target_elevation'], str(self.target_elevation))
        xp.setWidgetDescriptor(self.widget_dict['target_speed'], str(self.target_speed))
        xp.setWidgetDescriptor(self.widget_dict['target_heading'], str(self.target_trk))
        xp.setWidgetDescriptor(self.widget_dict['effective_angle'], str(self.effective_angle))
        xp.setWidgetDescriptor(self.widget_dict['closing_speed'], str(self.closing_speed))
        xp.setWidgetDescriptor(self.widget_dict['attack_valid'], str(self.attack_valid))
        xp.setWidgetDescriptor(self.widget_dict['RA'], str(self.RA_triggered))
        return 0.1  # Call this function again after 0.1 seconds