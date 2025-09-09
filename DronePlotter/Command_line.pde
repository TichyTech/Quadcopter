void keyPressed() {
  if (keyCode == ENTER) {
    sp.write(inputText + '\n');
    println("Sending text: ");
    println(inputText);
    lastInputText = inputText;  // Save the current input text
    inputText = "";             // Clear the input field
    cursorPos = 0;              // Reset cursor position
  }
  
  // Handle Arrow Up key: restore last input text
  else if (keyCode == UP) {
    inputText = lastInputText;  // Restore the last saved input text
    cursorPos = inputText.length();  // Set cursor to the end of the text
  }
  
  // Handle Arrow Left key: move cursor left (if not at the beginning)
  else if (keyCode == LEFT && cursorPos > 0) {
    cursorPos--;  // Move cursor position to the left
  }
  
  // Handle Arrow Right key: move cursor right (if not at the end)
  else if (keyCode == RIGHT && cursorPos < inputText.length()) {
    cursorPos++;  // Move cursor position to the right
  }
  
  // Handle Backspace key: remove the last character
  else if (keyCode == BACKSPACE) {
    if (inputText.length() > 0 && cursorPos > 0){
      inputText = inputText.substring(0, cursorPos - 1) + inputText.substring(cursorPos);  // Remove character at cursor position
      cursorPos--;  // Move cursor to the left after deleting
    }
  }
  
  else if (keyCode == '-'){
    println("Saving databases");
    ekf_database.save_to_file();
    state_database.save_to_file();
    att_database.save_to_file();
    quat_database.save_to_file();
  }
  
  // Handle other keys: add typed character to the input text
  else if (keyCode != SHIFT && keyCode != CONTROL && keyCode != ALT && keyCode != RIGHT && keyCode != LEFT) {
    inputText = inputText.substring(0, cursorPos) + key + inputText.substring(cursorPos);  // Insert character at the cursor
    cursorPos++;  // Move cursor position after the inserted character
  }
}


void drawCommandLine(int x, int y){
  fill(1);  // Light gray background for the input box
  stroke(0);
  rect(x, y-90, 500, 40, 10);  // Draw input box
  fill(0);  // Black color for text
  stroke(0);
  textAlign(LEFT, BASELINE);
  text(inputText, x+10, y-60);  // Display text inside the box
  float cursorX = textWidth(inputText.substring(0, cursorPos)) + 10; // Position based on the cursor position
  line(x+cursorX, y-55, x+cursorX, y-85);  // Draw a cursor
}
