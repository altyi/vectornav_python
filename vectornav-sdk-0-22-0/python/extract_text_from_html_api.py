import os
import re
from bs4 import BeautifulSoup, NavigableString

def extract_text_from_html(html_path, txt_path):
    """
    Reads an HTML file, cleans it by removing unwanted elements and reformatting
    code signatures, extracts the main content's visible text, and saves it
    to a text file.

    This function improves on simple text extraction by:
    - Focusing on the main content area of the documentation.
    - Removing script, style, and navigation link elements ('¶').
    - Consolidating multi-tag code signatures into single, readable lines.
    - Reformatting parameter, raises, and return blocks for better clarity.
    - Ensures the output directory exists before saving the text file.
    """
    try:
        # Step 1: Open and read the HTML file
        with open(html_path, 'r', encoding='utf-8') as file:
            html_content = file.read()

        # Step 2: Parse the HTML content using BeautifulSoup
        soup = BeautifulSoup(html_content, 'html.parser')

        # Step 3: Find the main content area to extract text from
        main_content = soup.find('div', role='main')

        if main_content:
            # --- Pre-processing the HTML tree for cleaner text extraction ---

            # a) Remove unwanted tags like scripts, styles, and header links (¶)
            for script_or_style in main_content(['script', 'style']):
                script_or_style.decompose()
            for header_link in main_content.find_all('a', class_='headerlink'):
                header_link.decompose()

            # b) Consolidate code signatures (<dt class="sig">) into single lines
            for dt in main_content.find_all('dt', class_='sig'):
                # .get_text() correctly handles embedded whitespace spans from the HTML
                signature_text = ' '.join(dt.get_text().split())
                # Replace complex inner tags with a single text node for clean extraction
                dt.clear()
                dt.append(NavigableString(signature_text))
            
            # c) Reformat field lists (like "Parameters:", "Raises:") for clarity
            for field_list in main_content.find_all('dl', class_='field-list'):
                field_items = []
                # Assuming dt and dd elements appear in pairs
                dts = field_list.find_all('dt', recursive=False)
                dds = field_list.find_all('dd', recursive=False)
                
                for dt, dd in zip(dts, dds):
                    dt_text = dt.get_text(strip=True)
                    # Consolidate whitespace in the description part
                    dd_text = ' '.join(dd.get_text(strip=True).split())
                    field_items.append(f"{dt_text} {dd_text}")
                
                # Replace the entire <dl> with a new <p> tag containing the formatted text
                new_p = soup.new_tag('p')
                new_p.string = '\n'.join(field_items)
                field_list.replace_with(new_p)

            # --- Extract text from the now-cleaned HTML tree ---
            text = main_content.get_text(separator='\n', strip=True)

            # d) Post-process the extracted text to normalize blank lines
            text = re.sub(r'\n{3,}', '\n\n', text)

        else:
            # Fallback for pages that don't have a <div role="main">
            print(f"⚠️  Warning: Could not find main content area in '{html_path}'. Extracting text from the whole page.")
            for script_or_style in soup(['script', 'style']):
                script_or_style.decompose()
            text = soup.get_text(separator='\n', strip=True)

        # Step 4: Ensure the output directory exists
        output_dir = os.path.dirname(txt_path)
        if output_dir: # Only try to create if there's a directory component
            os.makedirs(output_dir, exist_ok=True)
            
        # Step 5: Save the extracted text to a .txt file
        with open(txt_path, 'w', encoding='utf-8') as file:
            file.write(text)

        print(f"✅ Successfully extracted text from '{html_path}' to '{txt_path}'")

    except FileNotFoundError:
        print(f"❌ Error: The file '{html_path}' was not found.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


# --- Main execution block ---
if __name__ == "__main__":
    # --- Configuration ---
    # Define the input and output file names
    input_file = "documentation/api/registers.html"
    output_file = "extracted_html_content/registers.txt"
    
    # --- Run the extraction function ---
    extract_text_from_html(input_file, output_file)

    # Example with a different path to test directory creation
    # input_file_2 = "documentation/user_guide/index.html" # Assuming this file exists for testing
    # output_file_2 = "output/sub_dir/user_guide.txt"
    # extract_text_from_html(input_file_2, output_file_2)