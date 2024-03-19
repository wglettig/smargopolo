# RESTful API for NewGon:

**http Methods:**
- `GET` - get ressource
- `POST` - create a new ressource
- `DELETE` - deleting ressources



# GETTING information

## Getting the state of NewGon:

**Definition**

`GET /status

**Responses**
- `200 OK` - on success

```json
{
   "MODE":1,
   "STATUS":"Ready",
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
   "cyclecount":123456789
}
```

## Getting individual variables of NewGon:
**Definition**

`GET /status/<variable>`

**Response**
- `404 Not Found` if the variable does not exist
- `200 OK` on success

```json
{
   "MODE":1
}
```

# SETTING INFORMATION

## Setting many variables to NewGon 
**Definition**

`POST /status`

**Arguments**

```json
{ 
   "MODE":1,
   "STATUS":"Ready",
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
   "cyclecount":123456789
}
```

## Setting individual varibles to NewGon
**Definition**

`POST /status`

**Arguments**

```json
{
   "MODE":1,
   "SHX":2
}
```

{
  "SHX":2.2
  "message":"SHX->2.2."
}

