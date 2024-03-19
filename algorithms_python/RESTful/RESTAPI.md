# RESTful API for NewGon:

**http Methods:**
- `GET` - get ressource
- `POST` - create a new ressource
- `DELETE` - deleting ressources



# GETTING information

## Getting the full status of NewGon:

**Definition**

`GET /newgon

**Responses**
- `200 OK` - on success

```json
{
   "MODE":1,
   "target": {
       "SHX":0,
       "SHY":0,
       "SHZ":18,
       "CHI":10,
       "PHI":0,
       "OMEGA":0,
       "OX":0,
       "OY":0,
       "OZ":0,
       "s1":12,
       "s2":13,
       "s3":14,
       "s4":15,
       "phimotor":11,
       "omegamotor":180
   },
   "readback": {
       "s1":12,
       "s2":13,
       "s3":14,
       "s4":15,
       "phimotor":11,
       "omegamotor":180
    },
    "cyclecount":123456789,
    "status":"Ready"
}
```

## Getting target values of NewGon:
**Definition**

`GET /newgon/target`

**Response**
- `200 OK` on success

```json
{
    "target": {
       "SHX":0,
       "SHY":0,
       "SHZ":18,
       "CHI":10,
       "PHI":0,
       "OMEGA":0,
       "OX":0,
       "OY":0,
       "OZ":0,
       "s1":12,
       "s2":13,
       "s3":14,
       "s4":15,
       "phimotor":11,
       "omegamotor":180
    }
}
```

## Getting readback values of NewGon:
**Definition**

`GET /newgon/readback`

**Response**
- `200 OK` on success

```json
{
    "readback": {
       "s1":12,
       "s2":13,
       "s3":14,
       "s4":15,
       "phimotor":11,
       "omegamotor":180,
       "timestamp": "102020200"
   }
}
```



# SETTING INFORMATION

## Setting the operation MODE NewGon 
**Definition**

`POST /newgon/MODE`
`POST /newgon/mode`

**Arguments**
mode=1

or

```json
{ 
   "MODE":1,
}
```
**Responses**
- `200 OK` - on success
- `404 Not Found` if the variable does not exist


## Setting target values to NewGon 
**Definition**

`POST /newgon/target`

**Arguments**

```json
{ 
  "target": {
       "SHX":0,
       "SHY":0,
       "SHZ":18,
       "CHI":10,
       "PHI":0,
       "OMEGA":0,
       "OX":0,
       "OY":0,
       "OZ":0,
       "s1":12,
       "s2":13,
       "s3":14,
       "s4":15,
       "phimotor":11,
       "omegamotor":180
    }
}
```
**Responses**
- `200 OK` - on success
- `404 Not Found` if the variable does not exist


## Setting individual varibles to NewGon
**Definition**

`POST /newgon/target`

**Arguments**

```json
{
   "MODE":1,
   "SHX":2
}
```

```json
{
  "SHX":2.2,
  "message":"SHX->2.2."
}
```

# USING Command mode
For Streams, paths, etc.
**Definition**

`POST /newgon/mode?mode=5`
`POST /newgon/path`

**Arguments**

```json
{
   "gridO":{"start" : [0 1 1],
            "u" : [0.1 0 0],
            "v" : [0 0.1 0],
            "n_u" : 10,
            "n_v" : 1,
            "f" : 10
   },
   "gridSH": {"start" : [0 1 1],
            "u" : [0.1 0 0],
            "v" : [0 0.1 0],
            "n_u" : 10,
            "n_v" : 1,
            "f" : 10
   },
   "path": [
            {"OX":1,"OY":2,"OZ":3},
            {"OX":2,"OY":2,"OZ":1},
            {"OX":3,"OY":2,"OZ":1},
            {"OX":4,"OY":2,"OZ":2},
            {"OX":5,"OY":2,"OZ":3},
            {"OX":6,"OY":2,"OZ":4},
            {"OX":1,"OY":2,"OZ":3}
            ],
   "SHX":2
}
```




