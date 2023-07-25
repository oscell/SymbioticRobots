import pandas as pd
import matplotlib.pyplot as plt


# Load the data
data = pd.read_csv('CSV/Inspection.csv',on_bad_lines='skip')

# Convert the 'Author Keywords' to lowercase for uniformity
data['Author Keywords'] = data['Author Keywords'].str.lower()

# Filter rows containing the keyword 'digital twin'
data_dt = data[data['Author Keywords'].str.contains('inspection', na=False)]

# Group by 'Publication Year' and count the number of publications
dt_counts = data_dt['Publication Year'].value_counts().sort_index()

# Plot the data
plt.figure(figsize=(10, 6))
plt.plot(dt_counts.index, dt_counts.values, marker='o')
plt.title('Number of Publications Containing "Digital Twin" Over Time')
plt.xlabel('Year')
plt.ylabel('Number of Publications')
plt.grid(True)
plt.show()
plt.savefig('Plots/DT.png')
plt.close()

# Split the 'Author Keywords' column by semicolon
keywords = data_dt['Author Keywords'].str.split(';')

# Flatten the list of lists to get a single list of keywords
keywords = [item.strip() for sublist in keywords for item in sublist]

# Convert to a DataFrame
keywords_df = pd.DataFrame(keywords, columns=['Keyword'])

# Count the occurrences of each keyword
keyword_counts = keywords_df['Keyword'].value_counts()

# Exclude the phrase 'digital twin'
keyword_counts = keyword_counts[keyword_counts.index != 'inspection']

# Get the top 10 most common phrases
top_10_phrases = keyword_counts.head(20)

# Drop unwwanted phrases
# top_10_phrases = top_10_phrases.drop('digital twin (dt)')

# Plot the top 10 most common phrases
plt.figure(figsize=(10, 6))
top_10_phrases.plot(kind='bar', color='steelblue')
plt.title('Top 10 Most Common Phrases Used with "Digital Twin"')
plt.xlabel('Phrase')
plt.ylabel('Number of Occurrences')
plt.xticks(rotation=45, ha='right')
plt.tight_layout()
plt.show()
plt.savefig('Plots/DT_Phrases.png')
plt.close()

