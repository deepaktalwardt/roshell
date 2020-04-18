# Documentation

This is a list of features that we implemented along with a brief description on how to test them.

<hr>

## Interupting a program with Ctrl+C
The user can interrupt program execution with Ctrl+C
```
./roshell
sleep 10000
```

```sleep``` will be sent a SIGINT, and will exit. However, ```roshell``` does **not** exit.

<hr>

## Variable Handling

A variable can be assigned using '=' and retrieved with '$'
```
var=1
echo $var
1
```

<hr>
