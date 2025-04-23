import matplotlib.pyplot as plt

# Load the data from the text file
file_path = 'data.txt'

def load_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    data = []
    current_page = None
    for line in lines:
        line = line.strip()
        if line.startswith("Page:"):
            current_page = int(line.split()[1])
        elif current_page is not None:
            try:
                value = float(line)
                data.append(value)
            except ValueError:
                continue
    
    return data

def parse_data(data):
    x_data = []
    y_data = []
    entry_size = 8  # Each entry consists of 2 floats (4 bytes each), thus 8 bytes in total
    for i in range(0, len(data), 2):
        x_data.append(data[i])
        y_data.append(data[i+1])
    return x_data, y_data

def plot_data(x_data, y_data):
    plt.figure(figsize=(12, 6))
    
    # Plot x data
    plt.plot(range(len(x_data)), x_data, label='X duomenys', color='blue', alpha=0.7)
    
    # Plot y data
    plt.plot(range(len(y_data)), y_data, label='Y duomenys', color='red', alpha=0.7)
    
    plt.title('X ir Y duomenų grafikas')
    plt.xlabel('Laikas')
    plt.ylabel('Duomenys')
    plt.legend()
    plt.grid(True)
    plt.show()

data = load_data(file_path)
x_data, y_data = parse_data(data)
plot_data(x_data, y_data)