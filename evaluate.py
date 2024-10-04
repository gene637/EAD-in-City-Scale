import xml.etree.ElementTree as ET

def calculate_total_emissions_and_fuel(emission_file):
    # Parse the emission-export.xml file
    tree = ET.parse(emission_file)
    root = tree.getroot()

    total_co2_emission = 0.0
    total_fuel_consumption = 0.0

    # Iterate through each timestep
    for timestep in root.findall('timestep'):
        # Iterate through each vehicle entry
        for vehicle in timestep.findall('vehicle'):
            # Extract CO2 emissions and fuel consumption data
            co2_emission = vehicle.get('CO2')
            fuel_consumption = vehicle.get('fuel')
            
            # Accumulate CO2 emissions and fuel consumption
            if co2_emission is not None:
                total_co2_emission += float(co2_emission)
            if fuel_consumption is not None:
                total_fuel_consumption += float(fuel_consumption)

    return total_co2_emission, total_fuel_consumption

def calculate_total_travel_time(trip_file):
    # Load and parse the tripinfo.xml file
    tripinfo_tree = ET.parse(trip_file)
    tripinfo_root = tripinfo_tree.getroot()

    # Initialize total travel time
    total_travel_time = 0.0
    totalco2 = 0.0

    # Iterate through each tripinfo entry
    for tripinfo in tripinfo_root.findall('tripinfo'):
        duration = float(tripinfo.get('duration', 0.0))
        total_travel_time += duration
        for emissions in tripinfo.findall('emissions'):
            co2 = float(emissions.get('CO2_abs', 0.0))
            totalco2 += co2
    


    average_travel_time = total_travel_time / len(tripinfo_root.findall('tripinfo')) if len(tripinfo_root.findall('tripinfo')) > 0 else 0

    return total_travel_time, average_travel_time, totalco2

def calculate_vehicle_density(fcd_file):
    # Load and parse the fcd.xml file (Floating Car Data)
    fcd_tree = ET.parse(fcd_file)
    fcd_root = fcd_tree.getroot()

    # Initialize variables
    total_vehicle_count = 0
    total_length = 0.0
    road_length = 1000  # Example road length in meters, adjust as needed

    # Initialize a dictionary to store vehicle positions per timestep
    vehicle_positions = {}

    # Iterate through each timestep
    for timestep in fcd_root.findall('timestep'):
        time = float(timestep.get('time', 0.0))
        # Initialize vehicle count for this timestep
        vehicle_count = 0
        for vehicle in timestep.findall('vehicle'):
            pos = float(vehicle.get('pos', 0.0))
            if pos >= 0 and pos <= road_length:
                vehicle_count += 1
        total_vehicle_count += vehicle_count
        vehicle_positions[time] = vehicle_count

    # Calculate average density
    average_density = total_vehicle_count / (road_length * len(vehicle_positions))  # Vehicles per meter
    average_density_km = average_density * 1000  # Convert to vehicles per kilometer
    
    return average_density_km

def calculate_vht_vmt(tripinfo_file):

    tripinfo_tree = ET.parse(tripinfo_file)
    tripinfo_root = tripinfo_tree.getroot()

    # Initialize VMT and VHT
    total_vmt = 0.0
    total_vht = 0.0

    # Iterate through each tripinfo entry
    for tripinfo in tripinfo_root.findall('tripinfo'):
        route_length = float(tripinfo.get('routeLength'))
        duration = float(tripinfo.get('duration'))
        
        total_vmt += route_length
        total_vht += duration

    print(f"Total VMT: {total_vmt/1609.34} miles")
    print(f"Total VHT: {total_vht/3600} hours")





# Path to the emission-export.xml file
emission_file = '/home/gene/Downloads/EAD code test/trough_hour/EmissionOutput.xml'
trip_file = '/home/gene/Downloads/EAD code test/trough_hour/tripinfo.xml'
fcd_file = '/home/gene/Downloads/EAD code test/trough_hour/FCDOutput.xml'
tripinfo_file = '/home/gene/Downloads/EAD code test/trough_hour_0.1ev_static/tripinfo.xml'

# # Calculate total CO2 emissions and fuel consumption
# total_co2, total_fuel = calculate_total_emissions_and_fuel(emission_file)
# total_travel_time, average_travel_time, totalco2 = calculate_total_travel_time(trip_file)
# average_density_km = calculate_vehicle_density(fcd_file)

# # Print results
# print(f"Total CO2 emission: {total_co2} kg")
# print(f"Total fuel consumption: {total_fuel} kg")
# print(f"Total travel time: {total_travel_time} seconds")
# print(f"Average travel time: {average_travel_time} seconds")
# print(f"Average traffic density: {average_density_km} vehicles/km")
# print('co2', totalco2)

calculate_vht_vmt(tripinfo_file)
