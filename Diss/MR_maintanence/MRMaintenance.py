import matplotlib.pyplot as plt

# Data
labels = ['Aviation industry', 'Plant maintenance', 'Mechanical Maintenance', 'Consumer technology', 'Nuclear industry', 'Remote applications']
sizes = [17, 21, 29, 17, 8, 8]
colors = ['gold', 'yellowgreen', 'lightcoral', 'lightskyblue', 'lightgreen', 'lightpink']
explode = (0, 0.1, 0, 0, 0.1, 0)  # explode 1st and 4th slice for emphasis

# Plot
plt.figure(figsize=(10, 7))
plt.pie(sizes, explode=explode, labels=labels, colors=colors, autopct='%1.1f%%', shadow=True, startangle=140)
plt.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
plt.title("Field of application",fontweight='bold')

plt.savefig('Distribution of Various Industries.png')


# Data
labels = ['Dis/Assembly', 'Inspection and Diagnosis','Repair', 'Training' ]
sizes = [33,26, 26, 15]
colors = ['gold', 'yellowgreen', 'lightcoral', 'lightskyblue']
explode = (0, 0.1, 0, 0.1)  # explode 1st and 4th slice for emphasis

# Plot
plt.figure(figsize=(10, 7))
plt.pie(sizes, explode=explode, labels=labels, colors=colors, autopct='%1.1f%%', shadow=True, startangle=140)
plt.axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
plt.title("Maintenance Tasks",fontweight='bold')

plt.savefig('Distribution of Maintenance Tasks.png')