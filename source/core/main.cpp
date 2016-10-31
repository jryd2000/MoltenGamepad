#include <iostream>
#include "moltengamepad.h"
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <csignal>
#include <fstream>


int parse_opts(options& options, int argc, char* argv[]);
volatile bool STOP_WORKING = true;
volatile bool QUIT_APPLICATION = false;
moltengamepad* app;

void signal_handler(int signum) {
  if (!STOP_WORKING && signum != SIGHUP && signum != SIGPIPE) {
    //A long running action is to be interrupted.
    STOP_WORKING = true;
    return;
  }
  //Otherwise, we want to interrupt everything! (And let them shut down appropriately)
  STOP_WORKING = true;
  QUIT_APPLICATION = true;

  return;
}

void stdin_loop(moltengamepad* mg) {
  shell_loop(mg, std::cin);
  if (!mg->opts->get<bool>("stay_alive")) {
    QUIT_APPLICATION = true;
  }
}

int main(int argc, char* argv[]) {
  STOP_WORKING = true;
  QUIT_APPLICATION = true;
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);
  signal(SIGHUP, signal_handler);
  signal(SIGPIPE, signal_handler);

  int retcode = 0;
  options options;
  const option_decl* opt = &general_options[0];
  for (int i = 0; opt->name && *opt->name; opt = &general_options[++i]) {
    options.register_option(*opt);
  }
  int ret = parse_opts(options, argc, argv);

  if (ret > 0) return 0; //This was something like "--help" where we don't do anything.
  if (ret < 0) return ret;

  try {
    bool daemon = options.get<bool>("daemon");
    bool stay_alive = daemon || options.get<bool>("stay_alive");
    std::thread* stdin_thread = nullptr;
    int pid = -1;
    if (daemon)
      pid = fork();
    if (daemon && pid == -1) {
      std::cerr << "Failed to fork." << std::endl;
      throw -18;
    }
    if (pid > 0) {
      //We are a parent process! Just write out the pid and exit.
      std::string pidfile;
      options.get<std::string>("pidfile",pidfile);
      if (!pidfile.empty()) {
        std::ofstream pf;
        pf.open(pidfile);
        pf << pid << "\n";
        pf.close();
      }
      return 0;
    }
    //Now either we are the child process, or we never needed to fork.
    //Either way, time to get busy!

    //exit early for some errors.
    moltengamepad* mg;
    try {
      mg = new moltengamepad(&options);
      app = mg;
    } catch (int e) {
      return e;
    }
    //Any other errors will depend on us cleaning up that moltengamepad object.
    try {
      QUIT_APPLICATION = false;
      mg->init();

      //daemon doesn't have a useful STDIN.
      if (!daemon) {
        stdin_thread = new std::thread(stdin_loop, mg);
        stdin_thread->detach();
        delete stdin_thread;
        stdin_thread = nullptr;
      }

    } catch (...) {
      retcode = -45;
    }

    //Wait for the signal to quit.
    while (retcode == 0 && !QUIT_APPLICATION) {
      sleep(1);
    }
    delete mg;
    exit(0);

  } catch (int e) {
    return e;
  }

  exit(retcode);
}

int print_version() {
  std::cout <<  "MoltenGamepad version " << VERSION_STRING << "\n";
  return 0;
}

int print_usage(char* execname) {
  print_version();
  std::cout << "USAGE:\n";
  std::cout << "\t" << execname << " [OPTIONS]\n";
  std::cout << "\n";
  std::string help_text = ""\
                          "--help -h\n"\
                          "\tShow this message\n"\
                          "\n"\
                          "--version -v\n"\
                          "\tDisplay the version string\n"\
                          "\n"\
                          "--verbose -V\n"\
                          "\tDisplay extra information while running. Repeat this argument to show even more detail.\n"\
                          "\n"\
                          "--uinput-path -u\n"\
                          "\tSet where the uinput node is found on the system\n"\
                          "\n"\
                          "--make-fifo -m\n"\
                          "\tCreate a fifo command channel, and exit if it can't be made.\n"\
                          "\n"\
                          "--replace-fifo -m\n"\
                          "\tReplace the FIFO if it exists, while telling any existing listeners to exit.\n"\
                          "\n"\
                          "--fifo-path -f\n"\
                          "\tSet where the fifo command channel should be placed.\n"\
                          "\n"\
                          "--config-path -c\n"\
                          "\tSet a path for config files instead of $XDG_CONFIG_HOME/moltengamepad\n"\
                          "\t$XDG_CONFIG_DIRS is still respected.\n"\
                          "\n"\
                          "--profiles-path -p\n"\
                          "\tSet a path to find profiles before checking config directories\n"\
                          "\n"\
                          "--gendev-path -g\n"\
                          "\tSet a path to find generic driver descriptions before checking config directories\n"\
                          "\n"\
                          "--num-gamepads -n\n"\
                          "\tSet how many virtual gamepads will be created\n"\
                          "\n"\
                          "--no-make-keys\n"\
                          "\tDisable the creation of a virtual keyboard\n"\
                          "\n"\
                          "--no-enumerate\n"\
                          "\tDisable the search for already connected devices\n"\
                          "\n"\
                          "--no-monitor\n"\
                          "\tDisable listening for future connected devices\n"\
                          "\n"\
                          "--dpad-as-hat\n"\
                          "\tOutput dpad events as a hat rather than separate buttons\n"\
                          "\n"\
                          "--mimic-xpad\n"\
                          "\tMake the virtual output devices appear as xpad-style XBox 360 devices\n"\
                          "--rumble -R\n"\
                          "\tProcess controller rumble effects. (Do not quit while rumble effects are loaded.)\n"\
                          "--daemon -d\n"\
                          "\tFork and exit immediately, leaving the daemon running in the background.\n"\
                          "--pidfile -P\n"\
                          "\tOnly used for --daemon, gives a path for where to store the PID of the process.\n"\
                          "--stay-alive\n"\
                          "\tPrevents MoltenGamepad from shutting down when its standard input is closed.\n"\
                          ;

  std::cout << help_text;
  return 0;
}



int parse_opts(options& options, int argc, char* argv[]) {

  int c = 0;

  static struct option long_options[] = {
    {"help",          0,    0,  'h'},
    {"version",       0,    0,  'v'},
    {"uinput-path",   1,    0,  'u'},
    {"profiles-path", 1,    0,  'p'},
    {"gendev-path",   1,    0,  'g'},
    {"config-path",   1,    0,  'c'},
    {"num-gamepads",  1,    0,  'n'},
    {"make-fifo",     0,    0,  'm'},
    {"fifo-path",     1,    0,  'f'},
    {"pidfile",       1,    0,  'P'},
    {"no-make-keys",  0,    0,    0},
    {"no-enumerate",  0,    0,    0},
    {"no-monitor",    0,    0,    0},
    {"dpad-as-hat",   0,    0,    0},
    {"mimic-xpad",    0,    0,    0},
    {"daemon",        0,    0,  'd'},
    {"rumble",        0,    0,  'R'},
    {"verbose",       0,    0,  'V'},
    {"stay-alive",    0,    0,    0},
    {"replace-fifo",  0,    0,    0},
    {0,               0,    0,    0},
  };
  int long_index;

  //We lock the settings so that way command line args
  //take precedence over settings later read from files.
  while (c != -1) {
    c = getopt_long(argc, argv, "u:p:g:n:c:f:P:RmhvdV", long_options, &long_index);
    switch (c) {
    case 0:
      if (long_index == 10) {
        options.set("make_keyboard","false");
        options.lock("make_keyboard", true);
      };
      if (long_index == 11) {
        options.set("enumerate","false");
        options.lock("enumerate", true);
      };
      if (long_index == 12) {
        options.set("monitor","false");
        options.lock("monitor", true);
      };
      if (long_index == 13) {
        options.set("dpad_as_hat","false");
        options.lock("dpad_as_hat", true);
      };
      if (long_index == 14) {
        options.set("mimic_xpad","true");
        options.lock("mimic_xpad", true);
      };
      if (long_index == 18) {
        options.set("stay_alive","true");
        options.lock("stay_alive", true);
      };
      if (long_index == 19) {
        options.set("replace_fifo","true");
        options.lock("replace_fifo", true);
      };
      break;
    case 'd':
      options.set("daemon","true");
      options.lock("daemon", true);
      break;
    case 'u':
      options.set("uinput_path",std::string(optarg));
      options.lock("uinput_path", true);
      break;
    case 'm':
      options.set("make_fifo","true");
      options.lock("make_fifo", true);
      break;
    case 'f':
      options.set("fifo_path",std::string(optarg));
      options.lock("fifo_path", true);
      break;
    case 'p':
      options.set("profile_dir",std::string(optarg));
      options.lock("profile_dir", true);
      break;
    case 'P':
      options.set("pidfile",std::string(optarg));
      options.lock("pidfile", true);
      break;
    case 'g':
      options.set("gendev_dir",std::string(optarg));
      options.lock("gendev_dir", true);
      break;
    case 'c':
      options.set("config_dir",std::string(optarg));
      options.lock("config_dir", true);
      break;
    case 'R':
      options.set("rumble","true");
      options.lock("rumble", true);
      break;
    case 'h':
      print_usage(argv[0]);
      return 10;
      break;
    case 'v':
      print_version();
      return 10;
      break;
    case 'V':
      DEBUG_LEVEL++;
      if (*DEBUG_LEVEL < 0) //we reached past the end of our levels!
        DEBUG_LEVEL--;
      break;
    case 'n':
      options.set("num_gamepads",std::string(optarg));
      options.lock("num_gamepads", true);
      break;
    }

  }

  return 0;
}
