import random
import xml.etree.ElementTree as ET
import os
import optparse
import matplotlib.pyplot as plt


def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    optParser.add_option('--PR', type='int', default=10, help='CAEV penetration rate')
    optParser.add_option('--evPR', type='int', default=100, help='EV penetration rate')
    optParser.add_option('--RS', type='int', default=42, help='Random Seed number')
    optParser.add_option('--vcRat', type='int', default=10, help='VC ratio: 10, 67, 52, ...')
    optParser.add_option('--Hour', type='int', default=0, help='traffic hour: 0, 7, 10, 14, ...')
    options, args = optParser.parse_args()

    return options

def adjust_time_in_xaxis(s, old, new):
    # Replace the second occurrence by using split and join
    s = s[:2] + s[2:].replace(old, new)
    return s

# Convert keys to formatted strings with periods replaced by colons
def convert_key_to_time_string(num):
    
    # Get integer part (hours) and fractional part (minutes)
    hours = int(num)
    minutes = int(round((num - hours) * 100))
    
    # Format hours and minutes as two-digit strings
    return f"{hours:02d}.{minutes:02d}"

def plot_routes_24(data):
    # Sort the keys of the dictionary
    sorted_keys = sorted(float(key) for key in data.keys())
    # Extract the values corresponding to the sorted keys
    sorted_values = [data[convert_key_to_time_string(key)] for key in sorted_keys]

    # Convert keys to strings with periods replaced by colons
    x_labels = [convert_key_to_time_string(key).replace('.',':') for key in sorted_keys]

    # Plot the data
    plt.figure(figsize=(15, 6))  # Set the size of the figure
    sorted_times = sorted(float(adjust_time_in_xaxis(key,'3','5')) for key in data.keys())
    plt.plot(sorted_times, sorted_values, marker='o', linestyle='-', color='b')  # Plot the curve with markers and line

    # Set the x-ticks to the sorted keys and replace x-tick labels
    plt.xticks(ticks=sorted_times, labels=x_labels)

    # Rotate x-axis labels and adjust padding
    plt.xticks(rotation=45, ha='right')  # Rotate labels and align them to the right

    # Increase bottom margin to accommodate rotated labels
    plt.subplots_adjust(bottom=0.2)

    # Add title and labels to the plot
    plt.title('24 Hour Generated Traffic Volumes in Nagoya')  # Title of the plot
    plt.xlabel('Hours')  # Label for the x-axis
    plt.ylabel('Generated Volumes')  # Label for the y-axis

    # Show grid lines for better readability
    plt.grid(True)

    # Display the plot
    plt.show()

def generate_routefile_cav(path, newpath):
    global RandSeed, CAEVPr
    random.seed(RandSeed)

    veh_Nr = {}

    sub_items = os.listdir(path)
    for item in sub_items:

        # read XML file
        tree = ET.parse(path + item)
        root = tree.getroot()

        # read vehicle id range
        vehicles = root.findall('vehicle')
        if vehicles:
            first_vehicle_id = vehicles[0].get('id')
            last_vehicle_id = vehicles[-1].get('id')
        else:
            print("No vehicle elements found.")

        # calculate the number of vehicles generated in one route
        time = item[7:12].replace('_','.')
        veh_Nr.update({time: int(last_vehicle_id)-int(first_vehicle_id)+1})

        # modify vtype randomly
        for i in range(0, int(last_vehicle_id)-int(first_vehicle_id)+1):
            if random.uniform(0, 1) < CAEVPr:
                vehicles[i].set('type', 'EP')
            else:
                vehicles[i].set('type', 'EV')

        tree.write(newpath + item, encoding='utf-8', xml_declaration=True)
    
    # plot generated volumes in 24 hours
    # plot_routes_24(veh_Nr)

    # print route with max vehicles
    # for i in veh_Nr:
    #     print(i+':', veh_Nr[i])
    # max_key = max(veh_Nr, key=veh_Nr.get)
    # max_value = veh_Nr[max_key]
    # min_key = min(veh_Nr, key=veh_Nr.get)
    # min_value = veh_Nr[min_key]
    # print(max_key+': ', max_value)
    # print(min_key+': ', min_value)

def generate_routefile_ep(path, newpath):
    global RandSeed2, EPPr
    random.seed(RandSeed2)


    sub_items = os.listdir(path)
    for item in sub_items:

        # read XML file
        tree = ET.parse(path + item)
        root = tree.getroot()

        # read vehicle id range
        vehicles = root.findall('vehicle')
        if vehicles:
            first_vehicle_id = vehicles[0].get('id')
            last_vehicle_id = vehicles[-1].get('id')
        else:
            print("No vehicle elements found.")

        # modify vtype randomly
        for i in range(0, int(last_vehicle_id)-int(first_vehicle_id)+1):
            if random.uniform(0, 1) < EPPr:
                vehicles[i].set('type', 'EP')

        tree.write(newpath + item, encoding='utf-8', xml_declaration=True)

def main():


    path = '/home/gene/numo/routes/'
    new_path = '/home/gene/numo/new_routes_ep copy/'
    generate_routefile_cav(path, new_path)
    # generate_routefile_ep(path, new_path)

if __name__ == "__main__":
    options = get_options()
    print("options:")
    print(options)
    ###########################################
    # Global Constants
    RandSeed = options.RS
    RandSeed2 = 35
    CAEVPr = options.PR/100
    EPPr = 0.1
    ###########################################
    main()