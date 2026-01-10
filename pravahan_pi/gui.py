import plotly.express as px
from dash import Dash, dcc, html

app = Dash()
app.layout = [
    html.Div(children=[html.P("hello")]),
]

states = [2, 3, 5, 7]

if __name__ == "__main__":
    px.line(x=[0, 1], y=[states[0], states[0] + states[1] * (3.14 / 180)]).show()
    px.line(x=[0, 1], y=[0, states[2] * (3.14 / 180)]).show()
    px.line(x=[0, 1], y=[0, states[3] * (3.14 / 180)]).show()
