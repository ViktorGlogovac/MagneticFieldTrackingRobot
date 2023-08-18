#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>

struct termios original_termios;

// Function to disable buffered input
void disable_buffered_input() {
    struct termios new_termios = original_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    new_termios.c_cc[VMIN] = 1;
    new_termios.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

// Function to restore original terminal settings
void restore_original_input() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
}

int main() {
    char input;

    // Save original terminal settings
    tcgetattr(STDIN_FILENO, &original_termios);

    // Disable buffered input
    disable_buffered_input();
    atexit(restore_original_input);

    printf("Type characters and they will be echoed immediately. Press 'q' to quit.\n");

    while (1) {
        input = getchar();

        if (input == 'q') {
            break;
        }

        printf("You typed: %c\n", input);
    }

    return 0;
}
