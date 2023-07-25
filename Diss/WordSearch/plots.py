import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


# Define function to process data
def process_data(filename, keyword, plot_name, title, exclude):
    # Load the data
    data = pd.read_csv(filename, on_bad_lines='skip')

    # Convert the 'Author Keywords' to lowercase for uniformity
    data['Author Keywords'] = data['Author Keywords'].str.lower()

    # Filter rows containing the keyword
    data_filtered = data[data['Author Keywords'].str.contains(keyword, na=False)]

    # Group by 'Publication Year' and count the number of publications
    counts = data_filtered['Publication Year'].value_counts().sort_index()

    # Split the 'Author Keywords' column by semicolon
    keywords = data_filtered['Author Keywords'].str.split(';')

    # Flatten the list of lists to get a single list of keywords
    keywords = [item.strip() for sublist in keywords for item in sublist]

    # Convert to a DataFrame
    keywords_df = pd.DataFrame(keywords, columns=['Keyword'])

    # Count the occurrences of each keyword
    keyword_counts = keywords_df['Keyword'].value_counts()

    # Exclude the given phrase
    keyword_counts = keyword_counts[keyword_counts.index != exclude]

    return counts, keyword_counts

# Process the three files
counts_dt, keyword_counts_dt = process_data('CSV/DT.csv', 'digital twin', 'DT', 'Digital Twin', 'digital twin')
counts_inspection, keyword_counts_inspection = process_data('CSV/Inspection.csv', 'inspection', 'Inspection', 'Inspection', 'inspection')
counts_ar, keyword_counts_ar = process_data('CSV/AR.csv', 'augmented reality', 'AR', 'Augmented Reality', 'augmented reality')

# Plot the counts
plt.figure(figsize=(10, 6))
plt.plot(counts_dt.index, counts_dt.values, marker='o', label='Digital Twin')
plt.plot(counts_inspection.index, counts_inspection.values, marker='o', label='Inspection')
plt.plot(counts_ar.index, counts_ar.values, marker='o', label='Augmented Reality')
plt.title('Number of Publications Over Time')
plt.xlabel('Year')
plt.ylabel('Number of Publications')
plt.grid(True)
plt.legend()
plt.savefig('Plots/Publications.png')
plt.close()

# Find the common phrases among the top phrases in all three cases
common_phrases = set(keyword_counts_dt.index).intersection(set(keyword_counts_inspection.index)).intersection(set(keyword_counts_ar.index))

# Convert the common_phrases to a list
common_phrases = list(common_phrases)

# Create a DataFrame with the counts of the common phrases in each category
common_counts = pd.DataFrame({
    'Digital Twin': keyword_counts_dt[common_phrases],
    'Inspection': keyword_counts_inspection[common_phrases],
    'Augmented Reality': keyword_counts_ar[common_phrases]
}).fillna(0)

# Add a total column
common_counts['Total'] = common_counts.sum(axis=1)

# Sort by total counts
common_counts = common_counts.sort_values('Total', ascending=False)

# Plot the counts for the common phrases in each category
for column in common_counts.columns[:-1]:  # Exclude the total column
    plt.figure(figsize=(10, 6))
    common_counts[column].head(20).plot(kind='bar', color='steelblue')
    plt.title(f'Top 20 Common Phrases Used with "{column}" Sorted by Total Counts')
    plt.xlabel('Phrase')
    plt.ylabel('Number of Occurrences')
    plt.xticks(rotation=45, ha='right')
    plt.tight_layout()
    plt.savefig(f'Plots/{column}_Common_Phrases.png')
    plt.close()



# Compute the x coordinates for the groups
bar_width = 0.3
x = np.arange(len(common_counts))

# Create a figure and a set of subplots
fig, ax = plt.subplots(figsize=(12, 8))

# Generate the bars
rects1 = ax.bar(x - bar_width, common_counts['Digital Twin'], bar_width, label='Digital Twin')
rects2 = ax.bar(x, common_counts['Inspection'], bar_width, label='Inspection')
rects3 = ax.bar(x + bar_width, common_counts['Augmented Reality'], bar_width, label='Augmented Reality')

# Add labels, title and custom x-axis tick labels, etc.
ax.set_xlabel('Phrases')
ax.set_ylabel('Counts')
ax.set_title('Counts of Common Phrases by Category')
ax.set_xticks(x)
ax.set_xticklabels(common_counts.index, rotation=45, ha='right')
ax.legend()

# Add labels with the values to the top of the bars
def autolabel(rects):
    for rect in rects:
        height = rect.get_height()
        ax.annotate('{}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')

autolabel(rects1)
autolabel(rects2)
autolabel(rects3)

# Make layout tight
fig.tight_layout()

plt.show()

