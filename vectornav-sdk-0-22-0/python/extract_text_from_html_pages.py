import os
import re
from bs4 import BeautifulSoup

def extract_text_from_html(html_path, txt_path):
    """
    Reads an HTML documentation page, cleans it by removing unwanted elements,
    extracts the main content's visible text, and saves it to a text file.

    This script is designed for prose-style documentation pages ("pages")
    and is simpler than the API-specific extractor. It focuses on:
    - Extracting content from the main article body.
    - Removing scripts, styles, and navigation links ('¶').
    - Preserving the structure of headings, paragraphs, lists, and code blocks.
    - Ensuring the output directory exists before saving the text file.
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

            # b) Isolate and format code blocks to prevent them from being broken up.
            # We replace the entire highlighted div with a placeholder.
            for highlight_div in main_content.find_all('div', class_='highlight'):
                code_text = highlight_div.get_text()
                # Use a unique marker for later replacement
                placeholder_str = f"__CODE_BLOCK_START__\n{code_text.strip()}\n__CODE_BLOCK_END__"
                # Replace the div with a simple tag containing the placeholder string
                placeholder_tag = soup.new_tag("p")
                placeholder_tag.string = placeholder_str
                highlight_div.replace_with(placeholder_tag)

            # c) Add a unique newline marker after block-level elements.
            # This allows us to use a space as the main separator for get_text(),
            # which keeps inline elements together, and then restore newlines later.
            for block_element in main_content.find_all(['p', 'h1', 'h2', 'h3', 'li', 'ul', 'ol', 'div', 'blockquote']):
                block_element.append("__NEWLINE_MARKER__")

            # --- Extract text from the now-cleaned HTML tree ---
            # Use a space separator to correctly join text from inline elements.
            text = main_content.get_text(separator=' ', strip=True)

            # d) Replace markers with actual newlines and format code blocks.
            text = text.replace("__NEWLINE_MARKER__", "\n")
            text = re.sub(r'__CODE_BLOCK_START__\n?', '```python\n', text)
            text = re.sub(r'\n?__CODE_BLOCK_END__', '\n```', text)

            # e) Post-process the extracted text to normalize blank lines
            text = re.sub(r'(\s*\n\s*){2,}', '\n\n', text).strip()

        else:
            # Fallback for pages that don't have a <div role="main">
            print(f"⚠️  Warning: Could not find main content area in '{html_path}'. Extracting text from the whole page.")
            # Perform basic cleaning even in fallback mode
            for script_or_style in soup(['script', 'style']):
                script_or_style.decompose()
            for header_link in soup.find_all('a', class_='headerlink'):
                header_link.decompose()
            text = soup.get_text(separator='\n', strip=True)
            text = re.sub(r'\n{3,}', '\n\n', text)

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
        print(f"An unexpected error occurred while processing '{html_path}': {e}")


# --- Main execution block ---
if __name__ == "__main__":
    # --- Configuration ---
    # Define the input and output file names for a "page" document
    input_file = "documentation/pages/advanced_functionality.html"
    output_file = "extracted_html_content/advanced_functionality.txt"
    
    # --- Run the extraction function ---
    extract_text_from_html(input_file, output_file)