import functools
import ntcore
import wpilib

# Global publisher for alerts
_alert_pub = ntcore.NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/alert").publish()

class _LogTimer:
    """
    Encapsulates the 'Time Since Enabled' logic.
    Acts as a singleton source of truth for command logging timestamps.
    Internal use only.
    """
    def __init__(self):
        self._start_offset = 0.0
        self.reset()

    def reset(self):
        """Resets the timer to 0.0 relative to the current FPGA timestamp."""
        self._start_offset = wpilib.Timer.getFPGATimestamp()

    def get(self):
        """Returns the time in seconds since the last reset."""
        return wpilib.Timer.getFPGATimestamp() - self._start_offset

# Internal global instance
_log_timer = _LogTimer()

# Public API functions
def reset():
    """Resets the global log timer. Call this in autonomousInit and teleopInit."""
    _log_timer.reset()

def get_time():
    """Returns the current log time in seconds."""
    return _log_timer.get()

def log_command(cls=None, *, console=True, nt=False, print_init=True, print_end=True):
    """
    A class decorator that adds logging to the initialize and end methods of a command.
    Added to 2429 code in the off season of 2025 to make our commands simpler to read
    
    Usage:
        @log_command
        class MyCommand(Command): ...
        
        @log_command(console=True, nt=True)
        class MyCommand(Command): ...
    
    How it works:
    This function handles both usage styles by checking the 'cls' argument.
    1. @log_command: Python passes the class as the first argument ('cls'). 
       We process and return the class immediately.
    2. @log_command(...): Python calls this function first (cls is None). 
       We return the '_process_class' function. Python then calls that 
       returned function with the class.
    
    Args:
        console (bool): Whether to print to the console (default True).
        nt (bool): Whether to publish to SmartDashboard 'alert' (default False).
        print_init (bool): Whether to log the start message (default True).
        print_end (bool): Whether to log the end message (default True).
    """
    
    def _process_class(cls_inner):
        # Capture original methods
        orig_init = getattr(cls_inner, "initialize", lambda self: None)
        orig_end = getattr(cls_inner, "end", lambda self, interrupted: None)

        @functools.wraps(orig_init)
        def new_initialize(self):
            # --- Logging Start Logic ---
            # Set start_time for duration calculation later
            self.start_time = round(get_time(), 2)

            # --- Run Original Initialize First ---
            orig_init(self)

            if print_init:
                # Get attributes with defaults
                indent = getattr(self, "indent", 0)
                name = self.getName()
                
                # Check if the user wants to print extra info (e.g. target position)
                extra_info = getattr(self, "extra_log_info", "")
                if extra_info:
                    extra_info = f" with {extra_info}" 
                else:
                    extra_info = ""
                
                # Format the message
                msg = f"{'    ' * indent}** Started {name}{extra_info} at {self.start_time:.1f} s **"
                
                if console:
                    print(msg, flush=True)
                if nt:
                    _alert_pub.set(msg)

        @functools.wraps(orig_end)
        def new_end(self, interrupted: bool):
            # --- Run Original End ---
            orig_end(self, interrupted)
            
            # --- Logging End Logic ---
            if print_end:
                end_time = get_time()

                start_time = getattr(self, "start_time", end_time)
                duration = end_time - start_time
                
                message = 'Interrupted' if interrupted else 'Ended'
                indent = getattr(self, "indent", 0)
                name = self.getName()
                
                msg = f"{'    ' * indent}** {message} {name} at {end_time:.1f} s after {duration:.1f} s **"
                
                if console:
                    print(msg)
                if nt:
                    _alert_pub.set(msg)

        cls_inner.initialize = new_initialize
        cls_inner.end = new_end
        return cls_inner

    # Handle usage
    if cls is None:
        # Case: @log_command(console=...)
        # 'cls' is None because we were called as a function.
        # Return the wrapper function; Python will pass the class to it next.
        return _process_class
    elif isinstance(cls, type):
        # Case: @log_command (no parentheses)
        # 'cls' is the actual class. Process it immediately and return the modified class.
        return _process_class(cls)
    else:
        # Case: @log_command(True) - User passed a positional arg which got assigned to cls
        raise TypeError("Configuration arguments for @log_command must be passed as keywords (e.g. @log_command(console=True))")