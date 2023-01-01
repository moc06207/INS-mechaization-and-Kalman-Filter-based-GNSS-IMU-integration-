import csv
import numpy as np
import folium

def main():

    f = open('LLH_data.csv', 'r', encoding="UTF-8")
    rdr = csv.reader(f)
    data = []
    for line in rdr:
        row_list = []
        for i in range(len(line)):
            num = float(line[i])
            row_list.append(num)
        data.append(row_list)
    data = np.array(data)

    f.close()

    Span_data = data[:,:2]

    INS_data = data[:,3:5]

    center_x = sum(INS_data[:,0]) / len(INS_data[:,0])
    center_y = sum(INS_data[:,1]) / len(INS_data[:,1])

    INS_data = INS_data.tolist()
    Span_data = Span_data.tolist()

    center = [center_x, center_y]

    m = folium.Map(location=center,  zoom_start=100)

    folium.PolyLine(locations=INS_data, color = "red",).add_to(m)
    folium.PolyLine(locations=Span_data, color = "blue",).add_to(m)

    m.save("map.html")



    # folium.PolyLine(locations=meaningless_polyline[i], color='#ba1818').add_to(m)






if __name__ == '__main__':
    main()