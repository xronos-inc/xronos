# Monte Carlo Example

An application that approximates the value of π using Monte Carlo simulation and publishes the data to a webserver. Monte Carlo approximations are independent calculations that accumulate towards a converging value, which allows distribution of calculations across threads.

## Theory of Operation

The Monte Carlo technique for estimating the value of π (pi) is a probabilistic method that leverages randomness to solve problems that might be deterministic in principle but are nondeterministic in how they converge.

Imagine a square with a side length of 2, within which a circle with a radius of 1 is inscribed. The area of the square is 4 (since area = side²), and the area of the circle is π (since area = πr², and r = 1). By randomly generating points within the square, the ratio of points that fall inside the circle to the total number of points approximates the ratio of the circle's area to the square's area. Since the area of the square is 4 and the area of the circle is π, this ratio can be used to approximate π.

![animation of monte carlo approximation of π](docs/monte-carlo-pi.gif)

## Prerequisites

- xronos python library
- Python 3.10 or later
- Python virtual environment (optional but recommended)

## Run the Application

Install Python requirements.

```shell
pip install -r requirements.txt
```

Run the application with the parameter of the module and attribute to run for the webserver, in this case `app:app`.

```shell
uvicorn app:app
```

Launch the web app in your browser by navigating to the URL printed when the application starts. Note the default port address is `http://127.0.0.1:8000`. The port may be changed with commandline arguments; run `uvicorn --help` for details.

## Method

Initialization: Define a square with sides of length 2 and an inscribed circle of radius 1 centered at the origin (0,0).

Point Generation: Uniformly generate random points within the square. This means every point has an equal chance of being chosen anywhere in the square.

Counting: For each point, determine if it lies inside the circle. A point (x, y) is inside the circle if x²+y²≤1.

Calculating Pi: Calculate the ratio of points inside the circle to the total number of points and multiply by 4. This gives an approximation of π.

The effectiveness of the Monte Carlo method in approximating π relies on the law of large numbers, which states that as more trials are performed, the experimental probability (in this case, the ratio of points inside the circle to the total points) will converge to the theoretical probability. Thus, by increasing the number of random points, the approximation of π becomes more accurate.
