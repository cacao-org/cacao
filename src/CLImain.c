#include <stdio.h>
#include <stdlib.h>
#include <CommandLineInterface/CLIcore.h>


#define STYLE_BOLD    "\033[1m"
#define STYLE_NO_BOLD "\033[22m"



int main(int argc, char *argv[])
{
	char *AppName = "cacao";

	printf(STYLE_BOLD);
	printf("\n        Compute And Control for Adaptive Optics (cacao)\n");
	printf(STYLE_NO_BOLD);

	runCLI(argc, argv, AppName);

	return 0;
}
