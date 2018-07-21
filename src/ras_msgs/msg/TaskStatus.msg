uint8 status
uint8 PENDING     = 0   # Task has yet to be requested
uint8 ACTIVE      = 1   # Task is has started and is in progress
uint8 START       = 2   # Task start request
uint8 END         = 3   # Task end request
uint8 SUCCESS     = 4   # Task response successful
uint8 FAILED      = 5   # Task response failed/error

string text
